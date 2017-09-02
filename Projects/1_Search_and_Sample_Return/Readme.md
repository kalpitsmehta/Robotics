## Project 1: Search and Sample Return
### This project involves navigating a rover in a desert terrain while avoiding obstacles and locating the rock samples inside the map.
---

**Task 1: Identification of Navigation Terrain, Rock samples and Obstacles**

The first task in this project was to navigate the robot through the terrain. For this inside perception_step() function an appropriate RGB threshold was selected and applied to the prespective transformed image. One such result is shown below.

![Alt text](./Images/NavigableTerrain.PNG?raw=true "Navigable Terrain")

Next task was to isolate rocks and obstacles in the same manner as above. However, for easy of obstacle identification and thresholding prespect_transform() function was modified to accept border value and eventually isolate it during the thresholding phase.

![Alt text](./Images/Rocks.PNG?raw=true "Rocks")

![Alt text](./Images/Obstacles.PNG?raw=true "Obstacles")

**Task 2: Translate to different Coordinate systems**

For this purpose,first the threshold image pixels were converted to rover-centric coordinate system using rover_coords() function and nect to the world coordinate system using pix_to_world() function. Next, the rover-centric coordinates were translated to polar coordinates which will be used to make decisions later on in this project. All these information is upated into the Rover object so that it can be returned back to the calling function. While performing this task the threshold images were also updated in the Rover object so that they become visible on the left side of the simulation. Note: It is important to have this image in range 0 to 255 instead of 0 to 1 which will typically be the case with output of any threshold image.

**Task 3: Decision to navigate the terrain**

In this task, the decisions are to be maid for smooth navigation of the robot through the environment, find sample rock and obstacles. Following is the logic / tree of how this decisions were made.

![Alt text](./Images/P1 Decision Logic.png?raw=true "Decision")
