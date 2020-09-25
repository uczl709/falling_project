# Falling object mapping project

This the code, results and some models for this project.  There are six files.

evaluation_approximation: it contains the code for computing ICP scores/volume radio, viewing the mesh in point cloud and drawing the approximation shape. These method are mentioned in evaluation part.

original_third_party: it contains the original third party code for Supereight and simulation code. They can be found in https://bitbucket.org/smartroboticslab/workspace/projects/SI.

used_code: It contains the code used by me. Compared with the original_third_party, I changed some source code and used different setting.

modify_code: It only contains the codes that are different from the original_third_party.

object_model: It contains the ground truth model in my project. Rock2 is larger than 100Mb, so I do not upload it.

TSDFresult: It contains some reconstructed mesh for different experiment. I only upload the TSDF map result.



## Instructions for run

1. Run the gazebo_minimal_example to simulate the object falling. Change the falling object in the model.
2. Set the the map type, enable tracking, image topic in the Supereight_ros.
3. Run the Supereight_ros and save the result.
4. run the icp.cpp in evaluation to check the result and draw the approximation shape.

