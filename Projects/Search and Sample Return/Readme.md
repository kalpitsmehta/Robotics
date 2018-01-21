## Project 1: Search and Sample Return
#### This project involves navigating a rover in a desert terrain while avoiding obstacles and locating the rock samples inside the map.
---

### Training / Calibration

**Identification of Navigation Terrain, Rock samples and Obstacles**
The first step in this project is the identification of different pieces of images or in other words perception. As shown in pictures below. Few sample images are collected with terrain, rocks and obstacles in the scenario. Next, using the Matlibplot QT plot the pixel ranges for each of these objects are found. These values are then used to threshold the image so as to extract the pixel corresponding to either terrain, rock or obstacles respectively.

![Alt text](./Images/NavigableTerrain.PNG?raw=true "Navigable Terrain")

![Alt text](./Images/Rocks.PNG?raw=true "Rocks")

![Alt text](./Images/Obstacles.PNG?raw=true "Obstacles")

It is interesting to note that background parameter was introduced to the prespect_transform() function so as to accommodate inverting the obstacle colour channel (255-obstacle_img) 

**Translate to different Coordinate systems**

For this purpose, first, the threshold image pixels were converted to the Rover-centric coordinate system using rover_coords() function and next to the world coordinate system using pix_to_world() function. Next, the Rover-centric coordinates were translated to polar coordinates which will be used to make decisions later on in this project. All these information is updated into the Rover object so that it can be returned back to the calling function. While performing this task the threshold images were also updated in the Rover object so that they become visible on the left side of the simulation. Note: It is important to have this image in range 0 to 255 instead of 0 to 1 which will typically be the case with the output of any threshold image.

![Alt text](./Images/Transforms.PNG?raw=true "Transforms")

**Generating World Map and Video**

The first five steps for generating the World Map are completely same as the perception step to the point where information about world map coordinates is obtained. Once we have this coordinates it is used to update the world map with the information of terrain, obstacles and rocks. Next, a mosaic image is created which is segmented into original image in the upper left corner, the warped image in the upper right corner, a world map with ground truth and overlays in the lower left and finally some Rover statistics like position, orientation, throttle, velocity on the lower right. 

To create a video from this mosaic type images movipy.editor us used. The data information is reinitialized and fed at sixty frames per second to obtained the final output video clip. Which is further saved onto the local drive.

### Autonomous Navigation / Mapping

**Perception Step**

In this step first, the source and destination points for the perspective transform are defined. This information is then used to perform the perspective transform of the Rover image. Once these warped values are obtained they are passed through colour threshold function with an appropriate threshold value (as discussed in training section) for terrain, rocks and obstacles. To avoid calculating the horizon pixel for navigational terrain a predefined mask is applied. Parameters obtained from these operations i.e. navigable_terrain, rocks and obstacles are then used to update the Rover.vision_image. Further, the pixel values stored in these parameters are converted into Rover-centric co-ordinates and then to world coordinates and polar coordinates. Finally, the navigable distance and angles information for the Rover is returned to the Driver Rover section.

**Decision Step**

This steps takes in the Rover parameters as the input and completely focuses on decision making for navigation which is based on the information process in the perception step. On a high level, the Rover is asked to move if the values of navigational distance and angle are not null. If this is satisfied then the status of the Rover is checked if it's moving forward. While in the moving state the Rover is made to stop if the navigational angle is less than forwarding stop angel. Otherwise, the velocity of the Rover is verified. If this value is less than the maximum then the throttle for the Rover is set and its allowed to move forward. On the other hand, if the values of navigational distance and angels are null or small then the Rover is steered by a constant value added to the navigational angle. This helps Rover face in the direction where it can easily navigate. The complete decision tree is explained in a graphical format below.

![Alt text](./Images/P1_Logic.png?raw=true "Decision")
