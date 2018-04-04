# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from time import time
import operator

def handle_calculate_IK(req):
    """

    :param req: class 'kuka_arm.srv._CalculateIK.CalculateIKRequest'
    :return: CalculateIKResponse(joint_trajectory_list)
    """
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print('No valid poses received')
        return -1
    else:
        # Forward Kinematics Code
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i

        J0 = Point(0, 0, 0)
        J1 = Point(0, 0, 0.33)
        J2 = Point(0.35, 0, 0.75)
        J3 = Point(0.35, 0, 2)
        J4 = Point(1.31, 0, 1.946)
        J5 = Point(1.85, 0, 1.946)
        J6 = Point(2.043, 0, 1.946)
        JG = Point(2.153, 0, 1.946)

        # Modified DH parameters
        DH_Param ={	alpha0:	     0, a0:      0,	d1:  0.75, q1:	       q1,
                    alpha1: -pi/2., a1:   0.35,	d2:     0, q2: -pi/2 + q2,
                    alpha2:      0, a2:   1.25,	d3:     0, q3:		   q3,
                    alpha3: -pi/2., a3: -0.054,	d4:   1.5, q4:		   q4,
                    alpha4:  pi/2., a4:      0,	d5:     0, q5:         q5,
                    alpha5: -pi/2., a5:      0,	d6:     0, q6:         q6,
                    alpha6:      0, a6:      0,	d7: 0.303, q7:			0}

        # Modified DH Transformation matrix
        def DHTF(alpha, a, d, q):
            DHTF_Matrix= Matrix([[			cos(q),			  -sin(q),			  0,				a],
                               [ sin(q)*cos(alpha),	cos(q)*cos(alpha),	-sin(alpha),	-sin(alpha)*d],
                               [ sin(q)*sin(alpha),	cos(q)*sin(alpha),	 cos(alpha),	 cos(alpha)*d],
                               [				 0,					0,			  0,				1]])
            return DHTF_Matrix

        # Individual transformation matrices
        T0_1 = DHTF(alpha0, a0,d1,q1).subs(DH_Param)
        T1_2 = DHTF(alpha1, a1,d2,q2).subs(DH_Param)
        T2_3 = DHTF(alpha2, a2,d3,q3).subs(DH_Param)
        T3_4 = DHTF(alpha3, a3,d4,q4).subs(DH_Param)
        T4_5 = DHTF(alpha4, a4,d5,q5).subs(DH_Param)
        T5_6 = DHTF(alpha5, a5,d6,q6).subs(DH_Param)
        T6_G = DHTF(alpha6, a6,d7,q7).subs(DH_Param)

        T0_G = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

        r, p, y = symbols('r p y')

        ROT_x = Matrix([[1,      0,       0],
                        [0, cos(r), -sin(r)],
                        [0, sin(r), cos(r)]])

        ROT_y = Matrix([[cos(p),    0, sin(p)],
                        [	  0,	1,      0],
                        [-sin(p),	0, cos(p)]])

        ROT_z = Matrix([[cos(y),	-sin(y),	0],
                        [sin(y),	 cos(y),	0],
                        [	  0,		  0,	1]])

        ROT_EE = ROT_z * ROT_y * ROT_x

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

        ROT_EE = ROT_EE * Rot_Error

        #Initialize Additional Values
        side_a = round((J5.distance(J3)).evalf(),3)
        side_c = round((J2.distance(J3)).evalf(),3)
        angle_d = round(atan2(abs(a3), d4).subs(DH_Param), 3)

        # Initializing service response
        joint_trajectory_list = []
        error_list = []
        for x in xrange(0, len(req.poses)):
            start_time = time()
            # IK code
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Substitute to Compensate for rotation discrepancy between DH parameters and Gazebo
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (d7.subs(DH_Param)) * ROT_EE[:,2]

            # Calculating joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_a = side_a
            sideb_z = WC[2] - J2[2]
            sideb_xy = sqrt(WC[0] ** 2 + WC[1] ** 2) - J2[0]
            side_b = sqrt(sideb_xy ** 2 + sideb_z ** 2).evalf()
            # side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - J2[0].evalf()), 2) + pow((WC[2] - J2[2].evalf()),2))
            side_c = side_c

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            angle_d = angle_d

            theta2 = pi / 2 - angle_a - atan2(sideb_z, sideb_xy)
            theta3 = pi / 2 - (angle_b + angle_d)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3.inv("LU") * ROT_EE

            # Euler angles from rotation matrix while condidering the best solution
            theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            # theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            # theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
                theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            # Calculating FKs with thetas
            FK_x = T0_G[0, 3].subs({'q1': theta1, 'q2': theta2, 'q3': theta3, 'q4': theta4, 'q5': theta5, 'q6': theta6})
            FK_y = T0_G[1, 3].subs({'q1': theta1, 'q2': theta2, 'q3': theta3, 'q4': theta4, 'q5': theta5, 'q6': theta6})
            FK_z = T0_G[2, 3].subs({'q1': theta1, 'q2': theta2, 'q3': theta3, 'q4': theta4, 'q5': theta5, 'q6': theta6})

            # Calculating FK EE error for all poses
            Calc_EE = [FK_x, FK_y, FK_z]
            Actual_EE = [px, py, pz]
            EE_x_err = abs(Calc_EE[0] - Actual_EE[0])
            EE_y_err = abs(Calc_EE[1] - Actual_EE[1])
            EE_z_err = abs(Calc_EE[2] - Actual_EE[2])
            EE_offset = sqrt(EE_x_err ** 2 + EE_y_err ** 2 + EE_z_err ** 2)
            error_list.append(round(EE_offset,5))
            print("\nTime to calculate joint angle from pose is %04.4f seconds" % (time() - start_time))
        # Writing error list to file for plotting
        file = open("error.txt", "w")
        file.write(",".join(map(str, error_list)))
        file.close()
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        print("\nTotal run time to calculate last joint angles from pose is %04.4f seconds" % (time() - start_time))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print('Ready to receive an IK request')
    rospy.spin()

if __name__ == "__main__":
    IK_server()