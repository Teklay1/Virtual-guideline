# Virtual guideline for drone remote control

## Introduction 
This is to generate a virtual guideline for indoor drone teleoperation. 
## Point cloud
In this project, the Realsense L515 depth camera was employed to scan the environment and collect depth image data.
 
## Segmentation and down sampling 
Segmentation was achieved by classifying the point clouds from the same plane into homogeneous regions, grouping points at the same distance from the camera into corresponding regions. Additionally, a down-sampling process was implemented to decrease the number of points representing the actual environment, simplify computation.

## Mapping to 2D
The 3D point cloud data was converted to a 2D format to reduce the storage space required and streamline the execution process. Mathematical computations were then carried out using the transformed 2D point cloud data.

## Compute the virtual guideline
After completing the aforementioned process, a mathematical computation took place, resulting in the generation and projection of a virtual guideline at a specific distance from the origin of the point cloud data.

## LINE construction using PCL pointcloud data
The result is look like the following images:

![main](images/Actual_image_flatwall.png)
(Actual Image of the testing environment)
![main](images/Vertual_line_flatwall.png)<br /> 
(point clouds (green and blue points), generated vertual guideline (pink))
 
