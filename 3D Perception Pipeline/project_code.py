#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


# Function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

	# Converting ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Filter: Statistical Outlier
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(10)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()

    # Filter: Voxel Grid Downsampling
	vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # Filter: PassThrough - Z
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # Filter: PassThrough - X
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.3
    axis_max = 0.9
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # Segmentation: RANSAC Plane
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01  # 0.035
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extracting inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    outlier_filter = extracted_outliers.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(0)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    extracted_outliers = outlier_filter.filter()

    # Clustering: Euclidean
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)  # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    # New cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Converting PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # Publishing ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)
    pcl_rx_pub.publish(pcl_msg)
    pcl_objects_pub.publish(ros_cloud_objects)

    # Classifying the clusters
    detected_objects_labels = []
    detected_objects = []
    labeled_features = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = extracted_outliers.extract(pts_list)
        # Converting the cluster from pcl to ROS
        sample_cloud = pcl_to_ros(pcl_cluster)

        # Computing the associated feature vector
        # Extracting histogram features
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Making prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publishing into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud  # ros_cluster
        detected_objects.append(do)
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publishing the list of detected objects
    detected_objects_pub.publish(detected_objects)

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

def pr2_mover(object_list):
    # Initializing variables
    labels = []
    centroids = [] 
    dict_list = []
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    test_scene_num.data = 3

    object_list_param = rospy.get_param('/object_list')

    for i in range(0, len(object_list_param)):
        centroids = []
        object_name_i = object_list_param[i]['name']
        object_group_i = object_list_param[i]['group']

        for object in object_list:
            if (object_name_i == object.label):
                points_arr = ros_to_pcl(object.cloud).to_array()
                centroids.append(np.mean(points_arr, axis=0)[:3])
                place_pose_param = rospy.get_param('/dropbox')
                for j in range(0, len(place_pose_param)):
                    if (object_group_i == place_pose_param[j]['group']):
                        place_pose_arr = place_pose_param[j]['position']

                if (object_group_i == 'red'):
                    arm_name_i = 'left'
                elif (object_group_i == 'green'):
                    arm_name_i = 'right'

                object_name.data = object_list_param[i]['name']
                arm_name.data = arm_name_i
                pick_pose.position.x = np.asscalar(centroids[0][0])
                pick_pose.position.y = np.asscalar(centroids[0][1])
                pick_pose.position.z = np.asscalar(centroids[0][2])
                place_pose.position.x = place_pose_arr[0]
                place_pose.position.y = place_pose_arr[1]
                place_pose.position.z = place_pose_arr[2]
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                dict_list.append(yaml_dict)

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    # Sending request parameters into output yaml file
    send_to_yaml("output_3.yaml", dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    print("Initialized")

    # Creating Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Creating Publishers
    pcl_rx_pub = rospy.Publisher("/RxCloud", pc2.PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/objects", pc2.PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/table", pc2.PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/cluster", pc2.PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    model = pickle.load(
        open('/home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts/model_w3_250.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initializing color_list
    get_color_list.color_list = []

    # Wait for shutdown
    while not rospy.is_shutdown():
        rospy.spin()
