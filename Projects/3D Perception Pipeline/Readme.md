## Summary
This project focuses on creating a perception pipeline inside a ROS environment so as to enable a robot to perform a pick and place task for standard packaged products. Inspiration for this project is obtained from the Amazon Robotics Challenge. Challenges involved in this projects are isolating the product from surrounding, obtaining the location and centroid of the object, distinguishing the objects to define its type.

First task in this project is to obtain the camera input image. Below figure shows the input image capture by the camera on the head of the robot. As it can be seen there are three objects placed on a table. It can also be seen that there is some background noise capture by the camera.

![Image of RGBD](./images/Fig1_Raw_RGBD_Image.png)

### Statistical Outlier Filtering
First step is filtering point cloud data and remove the noise. For this purpose a Statistical Outlier Removal Filter is used. The results post filtering can be seen below.
![Image of SOF](./images/Fig2_SOF.png)

### Voxel Grid Down sampling
![Image of Voxel1](./images/Fig3.1_Voxel.png)
![Image of Voxel2](./images/Fig3.2_Voxel.png)

### RANSAC (Random Sample Consensus) Plane Fitting
![Image of RANSAC1](./images/Fig4.1_RANSAC-inliers.png)
![Image of RANSAC2](./images/Fig4.2_RANSAC-outliers.png)

### Clustering
![Image of Clustering](./images/Fig5_Clustering.png)

### Pass Through Filtering
![Image of PTF](./images/Fig6_PassThroughFilter.png)

### Feature Collection and Training
![Image of SVM](./images/Fig8_SVM_HOG.png)

### Object Detection
## Results
### World 1
![Image of World1](./images/Fig9.1_World1.png)

### World 2
![Image of World2](./images/Fig9.1_World2.png)

### World 3
![Image of World3](./images/Fig9.1_World3.png)
