## Project 1: Search and Sample Return
#### This project involves navigating a rover in a desert terrain while avoiding obstacles and locating the rock samples inside the map.
---

### Training / Calibration

**Task 1: Identification of Navigation Terrain, Rock samples and Obstacles**
The first step in this project is the identification of different pieces of images or in other words perception. As shown in pictures below. Few sample images are collected with terrain, rocks and obstacles in the scenario. Next, using the Matlibplot QT plot the pixel ranges for each of these objects are found. These values are then used to threshold the image so as to extract the pixel corresponding to either terrain, rock or obstacles respectively.

![Alt text](./Images/NavigableTerrain.PNG?raw=true "Navigable Terrain")

![Alt text](./Images/Rocks.PNG?raw=true "Rocks")

![Alt text](./Images/Obstacles.PNG?raw=true "Obstacles")

It is interesting to note that background parameter was introduced to the prespect_transform() function so as to accommodate inverting the obstacle colour channel (255-obstacle_img) 

**Task 2: Translate to different Coordinate systems**

For this purpose, first, the threshold image pixels were converted to the Rover-centric coordinate system using rover_coords() function and next to the world coordinate system using pix_to_world() function. Next, the Rover-centric coordinates were translated to polar coordinates which will be used to make decisions later on in this project. All these information is updated into the Rover object so that it can be returned back to the calling function. While performing this task the threshold images were also updated in the Rover object so that they become visible on the left side of the simulation. Note: It is important to have this image in range 0 to 255 instead of 0 to 1 which will typically be the case with the output of any threshold image.

**Task 3: Decision to navigate the terrain**

In this task, the decisions are to be made for smooth navigation of the robot through the environment, find sample rock and obstacles. Following is the flowchart describing how a decision is made.

![Alt text](./Images/P1_Logic.png?raw=true "Decision")

###Autonomous Navigation / Mapping
**Perception Step**
In this step first, the source and destination points for the perspective transform are defined. This information is then used to perform the perspective transform of the Rover image. Once these warped values are obtained they are passed through colour threshold function with an appropriate threshold value (as discussed in training section) for terrain, rocks and obstacles. To avoid calculating the horizon pixel for navigational terrain a predefined mask is applied. Parameters obtained from these operations i.e. navigable_terrain, rocks and obstacles are then used to update the Rover.vision_image. Further, the pixel values stored in these parameters are converted into Rover-centric co-ordinates and then to world coordinates and polar coordinates. Finally, the navigable distance and angles information for the Rover is returned to the Driver Rover section.

**Decision Step**
