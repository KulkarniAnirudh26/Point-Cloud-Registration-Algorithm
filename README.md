

### ROJECT OJECTIVE: 
To develop a pipeline for registering partial point cloud with 3D CAD model 

### STEPS INVOLVED WHILE RUNNING THE CODE: 
1. Make a BUILD folder, CMkeLists.txt and CODE.cpp in the directory you wish to run the code 
2. The arguments to be passses to the code are supposed to be place in build
3. The code accepts 3 arguments 
   Argument 1 = Kinect Point Cloud
   Argument 2 = CAD file 
   Argument 3 = Number of Iterations for ICP
4. Preprocessing for CAD file : CAD model, which is in .stl format and in milimeters is to be converted into .pcd and in meters. (Scalling the cad by 0.001)
5. Important Constraints for getting acceptable output: 
   a. The Kinect Position should not change as the transformation of Kinect with respect to robot base is hardcoded in the code.
   b. The Object to be registered is to be placed at a certain height. In the results obtained so far, the object was elevated above 6-7cm. (White small Cube)
      In the code, we have chopped of 
6. Example CMakelists, Matlab code to obtaing transformation of Kinect with respect to Base and Matalb code for obtaining transformation from Kinect to robot base are present in the zip file .
7. The code is to be compiled using "cmake .." and "make".

Variable names used in the code are mentioned in brackets 
CODE DESCRIPTION : 
(The code to run is FinaldCleaned)
The code can be divided in to three stages logically : 
1. Obtaining the point cloud of the object from the surroundings. 
   - This is achieved by chopping of the z values below and above a particularthreshold. 
   - In the code this is done by using filter and the values have been found empirically.

After this step, the Kinect image is transformed with respect to the Robot Base. (cloud_filtered/scene)

2. Robust Pose Estimation using Sample Consensus Prerejective 
   - The scene and the CAD file (object/origianl_cad) are downsampled
   - Normals of the scene are estimated
   - Normals and features of the CAD (object) and the scene are computed
   - These features are then used to get corresspondences between the target-Scene and the source-CAD model

Based on the the transformation obtained from Robust Pose Estimation, the CAD model (original_cad) is transformed into (transformed_cad) [transformation1]

3. ICP 
   - The input to ICP is the transforemd CAD and the scene. cloud_icp = transformed_cad, cloud_in = cloud_filtered

After ICP, another transformation matrix is found b/w transformed cad and the scene. [transformation_matrix]

The final transformation matrix (Final_Transformation) is formed by taking the product of transformation1 and transformation_matrix. 

The output of the code is in the form of final transformation matrix and visualization of the CAD convergene with the Scene. 
In the Visualization, 
White colour : It is used to represent CAD file 
Red colour : It is used to represent the final position of the CAD.
BLue clour: It is used to represent the scene/cloud_filtered.
Green colour :It is used to represent the CAD file transformed after Robust Pose Estimation. 



Miscellaneous : 
Kinect_to_Flang_transformation_matrix for Robot ABB= F_T_tcp_avg =   [  0.9912    0.1310   -0.0125   12.4498
 						 			0.0208   -0.0628    0.9977  -17.9773
								        0.1300   -0.9894   -0.0649  103.3841
					       				0         0         0    1.0000]


Sample point clouds captured using Kinect are placed in build folder. 

### Authors 
This Project has been done by: 
**Anirudh Kulkarni** - [KulkarniAnirudh26](https://github.com/KulkarniAnirudh26)
* **Siddhant Nadkarni** - [SiddhantNadkarni](https://github.com/SiddhantNadkarni)

