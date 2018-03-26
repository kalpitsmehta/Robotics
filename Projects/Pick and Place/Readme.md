## Summary
The project demonstrates a practical usecase of forward and inverse kinematics theory to control a 6 DoF Robotic Manipulator. The robotic arm is simulated in ROS environment, leveraging capabilities of gazebo and Rviz. The path planning for the Kuka robotic arm is achieved using a script on the server performing IK calculations.

## Kinematic Analysis to obtain DH parameters
Let's start by obtaining DH parameters. Run ‘forward_kinematics.py’ in the ROS environment. Extract robots parameters from the URDF file as shown in Table 1. This information is then used to create a sketch of the robotic arm with joint link and rotations (Figure 1). Modified DH parameter table is generated from figure 1 and is populated in Table 2.

#### Table 1 Information extracted from the URDF File

| Joint  | Origin XYZ |
| ------ | ------------- |
| 1      | (0, 0, 0.33)  |
| 2      | (0.35, 0, 0.42) |
| 3      | (0, 0, 1.25) |
| 4      | (0.96, 0, -0.0054) |
| 5      | (0.193, 0, 0) |
| 6      | (0.193, 0, 0) |

#### Table 2 Modified DH Parameters
 
