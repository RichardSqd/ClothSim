#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double unitH = height/(double)num_height_points;
  double unitV = width/(double)num_width_points;
  if(num_height_points==0 || num_width_points==0) return;
  srand((unsigned)time(0));
  for(int i=0; i<num_height_points;i++){
      for(int j=0; j<num_width_points; j++){
          Vector3D position;
          bool pinned= false;
          if(orientation==HORIZONTAL){
              position.y=1;
              position.x = (double)j * unitV;
              position.z = (double)i * unitH;
          }else{
              double r = (double)rand()/RAND_MAX;
              position.z = -1.0/1000.0 + r * (2.0/1000.0);
              position.x = (double)j * unitV;
              position.y = (double)i * unitH;

          }
          for(auto k= this->pinned.begin();k!=this->pinned.end();++k){
              if(j==(*k)[0] && i==(*k)[1]) {
                  pinned = true;
              }
          }
          this->point_masses.emplace_back(position,pinned);
      }
  }
  for(int i=0; i<num_width_points; i++){
      for(int j=0; j<num_height_points;j++){
          //structural
          if(i<num_width_points-1) this->springs.emplace_back(&this->point_masses[i*num_height_points+j],&this->point_masses[(i+1)*num_height_points+j],STRUCTURAL);
          if(j<num_height_points-1) this->springs.emplace_back(&this->point_masses[i*num_height_points+j],&this->point_masses[i*num_height_points+j+1],STRUCTURAL);

          //shearing
          if(i<num_width_points-1 && j<num_height_points-1 ) this->springs.emplace_back(&this->point_masses[i*num_height_points+j],&this->point_masses[(i+1)*num_height_points+(j+1)],SHEARING);
          if(j<num_height_points-1 && i<num_width_points-1) this->springs.emplace_back(&this->point_masses[(i+1)*num_height_points+j],&this->point_masses[i*num_height_points+(j+1)],SHEARING);

          //bending
          if(i<num_width_points-2) this->springs.emplace_back(&this->point_masses[i*num_height_points+j],&this->point_masses[(i+2)*num_height_points+j],BENDING);
          if(j<num_height_points-2) this->springs.emplace_back(&this->point_masses[i*num_height_points+j],&this->point_masses[i*num_height_points+(j+2)],BENDING);

      }
  }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  //(Part 2): Compute total force acting on each point mass.
  if(mass==0) return ;
  //external forces
  Vector3D externalAcc, externalForces;
  for(auto j = external_accelerations.begin(); j!=external_accelerations.end();++j){
      externalAcc += (*j);
  }
  externalForces = externalAcc * mass;

  for(auto i = this->point_masses.begin(); i!=this->point_masses.end(); ++i){
      (*i).forces = externalForces;
  }

  //spring correction forces
  for(auto sp = this->springs.begin(); sp!=this->springs.end(); ++sp){
      if( (*sp).spring_type == STRUCTURAL && cp->enable_structural_constraints){
          Vector3D force = cp->ks  * ( ((*sp).pm_a->position-(*sp).pm_b->position).norm()  -(*sp).rest_length)*((*sp).pm_a->position-(*sp).pm_b->position).unit();
          (*sp).pm_a->forces -= force;
          (*sp).pm_b->forces += force;
      }
      else if( (*sp).spring_type == SHEARING && cp->enable_shearing_constraints){
          Vector3D force = cp->ks * ( ((*sp).pm_a->position-(*sp).pm_b->position).norm()  -(*sp).rest_length)*((*sp).pm_a->position-(*sp).pm_b->position).unit();
          (*sp).pm_a->forces -= force;
          (*sp).pm_b->forces += force;
      }
      else if((*sp).spring_type==BENDING && cp->enable_bending_constraints){
          Vector3D force = cp->ks * 0.2 * ( ((*sp).pm_a->position-(*sp).pm_b->position).norm()  -(*sp).rest_length)*((*sp).pm_a->position-(*sp).pm_b->position).unit();
          (*sp).pm_a->forces -= force;
          (*sp).pm_b->forces += force;
      }
  }


  // Use Verlet integration to compute new point mass positions
  //Verlet integration
    for(auto pm = this->point_masses.begin(); pm!=this->point_masses.end(); ++pm) {
        if(!(*pm).pinned) {
            Vector3D new_position =
                    (*pm).position + (1 - cp->damping / 100.0) * ((*pm).position - (*pm).last_position) +
                    delta_t * delta_t * (*pm).forces / mass;
            (*pm).last_position = (*pm).position;
            (*pm).position = new_position;
        }
    }


    build_spatial_map();

    for(auto pm = this->point_masses.begin(); pm!=this->point_masses.end(); ++pm) {
        self_collide(*pm,simulation_steps);
        for(auto co = collision_objects->begin(); co!= collision_objects->end(); ++ co){
            (*co)->collide(*pm);
        }
    }

    // in length more than 10% per timestep [Provot 1995].
    //update
    for(auto sp = this->springs.begin(); sp!=this->springs.end(); ++sp){
        if(!(*sp).pm_a->pinned || !(*sp).pm_b->pinned){
            double len = ((*sp).pm_a->position - (*sp).pm_b->position).norm();
            double correction = len - 1.1*(*sp).rest_length;
            if(correction  > 0){
                if((*sp).pm_a->pinned){

                    Vector3D newb = (*sp).pm_b->position + (correction/len)*((*sp).pm_a->position - (*sp).pm_b->position);
                    (*sp).pm_b->position = newb;
                }else if((*sp).pm_b->pinned){
                    Vector3D newa = (*sp).pm_a->position + (correction/len)*((*sp).pm_b->position - (*sp).pm_a->position);
                    (*sp).pm_a->position = newa;
                }else{
                    Vector3D newa = (*sp).pm_a->position + (correction/2/len)*((*sp).pm_b->position - (*sp).pm_a->position);
                    Vector3D newb = (*sp).pm_b->position + (correction/2/len)*((*sp).pm_a->position - (*sp).pm_b->position);
                    (*sp).pm_a->position = newa;
                    (*sp).pm_b->position = newb;

                }
            }

        }
    }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for(auto pm = this->point_masses.begin(); pm!=this->point_masses.end(); ++pm) {
      double k = hash_position((*pm).position);

      if(!map[k])
      {
          map[k] = new vector<PointMass *>();
      }
      map[k]->push_back(&(*pm));
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  double k = hash_position(pm.position);
  double dis;
  Vector3D corr,diff;
  int num = 0;
  if(map[k]){
      for(auto it=map[k]->begin();it!=map[k]->end();++it){
          diff = pm.position - (*it)->position;
          dis = 2*thickness-diff.norm();
          if((*it)!=&pm && dis>=0){
              corr += diff.unit() * dis;
              num++;
          }
      }
  }
  if(num>0){
      pm.position += corr/num/simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3.0 * width / (double)num_width_points;
  double h = 3.0 * height / (double)num_height_points;
  double t = max(w,h);
  int x =  floor (pos.x / w);
  int y = floor(pos.y / h);
  int z = floor(pos.z / t);
  //fmod(x,y);
  return x+y*num_width_points/3.0+z*(num_width_points/3.0)*(num_height_points/3.0);
  //return x * 31 * 31 + y * 31 + z;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */

      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;

      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;

      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);


      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
