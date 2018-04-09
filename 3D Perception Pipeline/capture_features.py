#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'sticky_notes',
       'book',
       'snacks',
       'biscuits',
       'eraser',
       'soap2',
       'soap',
       'glue']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    for model_name in models:
        spawn_model(model_name)
        #Print Object Name and capture multiple images of it at different orientation.
        print(model_name)
        for i in range(250):
	    print(i)
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

	    cloud = ros_to_pcl(sample_cloud)
	    # Voxel Grid Downsampling
	    # Create a VoxelGrid filter object for input point cloud
	    vox = cloud.make_voxel_grid_filter()

	    # Choose a voxel (also known as leaf) size
	    # Note: this (1) is a poor choice of leaf size
	    # Experiment and find the appropriate size!
	    LEAF_SIZE = 0.01

	    # Set the voxel (or leaf) size
	    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

	    # Call the filter function to obtain the resultant downsampled point cloud
	    cloud_filtered = vox.filter()
	    sample_cloud = pcl_to_ros(cloud_filtered)

        # Extract histogram features
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        labeled_features.append([feature, model_name])

        delete_model()
    pickle.dump(labeled_features, open('training_set.sav', 'wb'))

