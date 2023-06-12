Cloth Simulator
================

This project is a cloth simulator based on a mass and spring system. There are three main steps involved in achieving this simulation. Firstly, an evenly spaced grid of masses is constructed, forming the foundation of the cloth simulation.
Next, the realistic behavior of cloth is implemented by translating mathematical and physics models into code, enabling accurate cloth movement and dynamics. Finally, to enhance the visual effects, various shaders such as diffuse and Blinn-Phong are developed. These shaders improve the rendering of the cloth, adding realistic lighting and material properties to enhance the overall visual appearance.

![](images/banner.png)

Implementation Notes:
---

<details>
<summary><b>Section 1: Masses and springs</b></summary>

<b>Cloth::buildGrid</b> constructs an evenly spaced grid of masses. In the first loop, all point masses are created, and the orientation of the cloth is checked. If its value is horizontal, the y-coordinate is set to 1, and the x and z positions are assigned over the xz plane. Otherwise, the z-coordinate is set to a random number within the range [-1/1000, 1/1000], while the x and y coordinates are assigned over the xz plane. If the (x,y) position of a point mass is within the cloth's pinned vector, the "pinned" variable is set to true.

In the second loop, springs are created to enforce constraints between point masses. Within each iteration, structural, shearing, and bending constraints are applied respectively and checked for edge cases to avoid segmentation faults. Once these steps are completed, a horizontal flat cloth wireframe is visible (see figure 1). Additionally, certain constraints can be enabled or disabled (see figures 3, 4, and 5).

<img src="images/task1_1.png"  width="480"><br>
Pinned2 image 1


<img src="images/task1_Noshearing.png" width="480"><br>
Pinned2 image 3: without any shearing 

<img src="images/task1_Onlyshearing.png"  width="480"><br>
Pinned2 image 4: with only shearing constraints 

<img src="images/task1_all.png"  width="480"><br>
Pinned2 image 5: with all constraints

</details>


<details>
<summary><b>Section 2: Simulation via numerical integration</b></summary>

<p> Now, physical equations are applied to determine the motion of each point mass in every time step. <b>Cloth::simulate()</b> executes one time step for the simulation.</p>
<p> In this implementation, two types of forces are taken into account: external forces and spring correction forces. The total external force is calculated by applying a force formula to the point masses vector. Subsequently, spring correction forces (based on Hooke's law) are applied to the springs vector. It's important to note that the constraint status is checked before applying the forces. This is necessary to ensure that there are two opposing forces for each point mass connected by a single spring. </p>
<p> In the second section, Verlet integration is employed to calculate new positions for the point masses. Additional steps are taken to prevent spring deformation based on the SIGGRAPH 1995 Provot paper. At the end of each time step, we specifically adjust springs that are 10% longer than their rest length. It is safe to skip this adjustment if both point masses connected by a spring are pinned, as their positions remain static. However, if one of the point masses is pinned, we only need to modify the position of the other point mass. For example if a is pinned, the corrected position of b is given by <i>b = b + diff/||a-b||*(a-b)</i> , where diff is defined by <i>diff = length - 1.1* rest_length</i></p>


<p>The spring constant <b>ks</b> represents the internal forces between point messes. A high ks value makes the movement of cloth more restricted and it's easy to observe the difference in the figures below. </p>

<img src="images/t2_lowks.png" width="480px" /><br>
Pinned2 low ks (more relaxed)

<img src="images/t2_highks.png" width="480px" /><br>
Pinned2 high ks (more restricted)


<p>The density determines how forces affect the cloth as mass is directly proportional to density. For instance, with higher density, we observe that the middle portion of the cloth hangs lower. On the other hand, with lower density, the middle section aligns more closely with the two pinned points.</p>

<img src="images/t2_lowdensity.png" width="480px" /><br>
Pinned2 low density

<img src="images/t2_highdensity.png" width="480px" /><br>
Pinned2 high density

<p> The damping parameter determines the extent to which the cloth sways. When the damping value is low, the cloth continues to swing back and forth. However, with high damping, the cloth experiences minimal swinging or oscillation.</p>

<img src="images/t2_dampinglow.png" width="480px" /><br>
Pinned2 low damping 

<img src="images/t2_damplinghigh.png" width="480px" /><br>
Pinned4 high damping

<img src="images/task2_pinned4_1.png" width="480px" /><br>
 Pinned4 image 1 

 <img src="images/task2_pinned4_2.png" width="480px" /><br>
 Pinned4 image 2
</details>


<details>
<summary><b>Section 3: Handling collisions with other objects</b></summary>

In this section, cloth collision with spheres and planes is implemented. For Sphere::collide, the positions of point masses that interact with or are inside the sphere are adjusted. First, the distance between the point mass and the center of the sphere is calculated to determine if any changes are necessary. Then, the tangent point is computed, which represents where the point mass should have intersected the sphere. The correction vector is then determined by subtracting the last_position vector from the tangent vector. Finally, the friction formula is applied to the correction vector to obtain the new position.

To prevent point masses from crossing the plane from one side to the other, collision detection is performed for Plane::collide. If a collision is detected, the correction vector, similar to the previous task, is added to the last position of the point mass to move both points to the same side of the plane.

The accompanying images are rendered with ks values of 500, 5000, and 50000. When ks is set to 500, the cloth hangs more compared to the other two images. This is because the internal forces within the springs are smaller, resulting in less restriction on the cloth's movement.

<img src="images/t3_500.png" width="480px" /><br>
Sphere collision ks = 500

<img src="images/t3_5000.png" width="480px" /><br>
Sphere collision ks = 5000

<img src="images/t3_50000.png" width="480px" /><br>
Sphere collision ks = 50000

<img src="images/t3_plane2.png" width="480px" /><br>
Cloth on the plane 1

<img src="images/t3_plane1.png" width="480px" /><br>
Cloth on the plane 2


</details>

<details>
<summary><b>Section 4: Handling self-collisions</b></summary>

<p>Self-collisions are implemented in this section. In Cloth::hash_position, unique floats are calculated for the hash table. Each individual 3D volume box is indexed using the formula x + yN + zN*N, where N represents num_width_points/3.0.

Cloth::self_collide is then implemented to compute the hash value of a point mass and find all other point masses that are within a distance of 2*thickness. The corresponding correction vector is calculated for each collision. To correct the new position, the correction vector is applied, scaled down by the value of simulation_steps.
</p>

<img src="images/t4_1.png" width="480px" /><br>
Self collision 1

<img src="images/t4_2.png" width="480px" /><br>
Self collision 2

<img src="images/t4_3.png" width="480px" /><br>
Self collision 3

<img src="images/t4_4.png" width="480px" /><br>
Self collision 4


<p>The following images are rendered with different density and ks values. When ks is high or density is low, the cloth tends to retain its original shape. Conversely, when ks is low or density is high, the cloth exhibits more folding. </p>

<img src="images/t4_1.png" width="480px" /><br>
Self collision density =15, ks =5000

<img src="images/t4_5.png" width="480px" /><br>
Self collision density =15, ks = 1

<img src="images/t4_6.png" width="480px" /><br>
Self collision density = 1, ks =5000

<img src="images/t4_7.png" width="480px" /><br>
Self collision density = 1, ks = 10000


</details>


<details>
<summary><b>Section 5: Shaders</b></summary>
<p> In this section, several GLSL shaders are implemented. Specifically, the Blinn-Phong reflection model is implemented, which includes specular highlights, diffuse reflection, and ambient lighting. For the specular component, it represents the reflection of the light source at positions where the surface is curved in a way that the mirror reflection direction is close to the light source. The half vector, denoted as H = L + V / |L + V|, is introduced in this context. </p>

<img src="images/t5_ambient.png" width="480px" /><br>
Blinn-Phong ambient only 

<img src="images/t5_specular.png" width="480px" /><br>
Blinn-Phong specular only

<img src="images/t5_diffuse.png" width="480px" /><br>
Blinn-Phong diffuse only

<img src="images/t5_full.png" width="480px" /><br>
Blinn-Phong entire

<p>Custom texture:</p>
<img src="images/t5_texture1.png" width="480px" /><br>
Custom texture 1

<img src="images/t5_texture2.png" width="480px" /><br>
Custom texture 2


<p> Bump mapping and displacement mapping shaders are also implemented, utilizing the texture_3.png for rendering. In comparison, the displacement shader produces nearly identical outputs for two different resolutions, while the bump mapping shader only provides satisfactory results in high resolution. This discrepancy arises because the bump mapping shader merely simulates the appearance of surface bumpiness by reorienting surface normals. In contrast, displacement mapping alters the geometry of a surface. Consequently, aliasing occurs when bump mapping is employed under low resolution, while displacement mapping maintains its quality.</p>

<img src="images/t5_displacement16.png" width="480px" /><br>
 displacement resolution 16*16

<img src="images/t5_displacement128.png" width="480px" /><br>
displacement resolution 128*128


<img src="images/t5_bump16.png" width="480px" /><br>
Bumping resolution 16*16

<img src="images/t5_bump128.png" width="480px" /><br>
Bumping resolution 128*128

<img src="images/t5_displacement.png" width="480px" /><br>
Cloth displacement

<img src="images/t5_bump.png" width="480px" /><br>
Cloth bumping

<img src="images/t5_mirror1.png" width="480px" /><br>
Sphere

<img src="images/t5_mirror2.png" width="480px" /><br>
Cloth on sphere
</details>