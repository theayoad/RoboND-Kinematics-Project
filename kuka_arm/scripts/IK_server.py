#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.
# Author: Harsh Pandya
import numpy as np
import rospy
import tf
from angles import shortest_angular_distance_with_limits
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import GetModelProperties, GetJointProperties
from geometry_msgs.msg import Pose
from math import degrees, radians, hypot
from mpmath import *
from sympy import *


# define DH symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')


# create DH table
dh  = {
    alpha0:        0, a0:      0, d1:  0.75, q1:             q1,
    alpha1: -np.pi/2, a1:   0.35, d2:     0, q2:   q2 - np.pi/2,
    alpha2:        0, a2:   1.25, d3:     0, q3:             q3,
    alpha3: -np.pi/2, a3: -0.054, d4:  1.50, q4:             q4,
    alpha4:  np.pi/2, a4:      0, d5:     0, q5:             q5,
    alpha5: -np.pi/2, a5:      0, d6:     0, q6:             q6,
    alpha6:        0, a6:      0, d7: 0.303, q7:              0}


def TF_matrix(alpha, a, d, sq, cq):
    """
    Generate homogeneous transform from frame {i-1} to {frame i}
    using DH parameters descriptive of frame{i-1} and frame {i}
    Inputs:
        alpha (symbol) # twist angle
        a (symbol) # link length
        d (symbol) # link offset
        sq (symbol) # sin(theta)
        cq (symbol) # cos(theta)
    Returns:
        TF (Matrix)
    Returns:
    """
    TF = Matrix([
        [           cq,           -sq,           0,             a],
        [sq*cos(alpha), cq*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sq*sin(alpha), cq*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [             0,            0,           0,             1]])
    return TF

# create individual transformation matrices
alpha, a, d, sq, cq = symbols('alpha a d sq cq')
Ti_j = lambdify([alpha, a, d, sq, cq], TF_matrix(alpha, a, d, sq, cq))
T0_1 = Matrix(Ti_j(dh[alpha0], dh[a0], dh[d1], sin(q1), cos(q1)))
T1_2 = Matrix(Ti_j(dh[alpha1], dh[a1], dh[d2], sin(dh[q2]), cos(dh[q2])))
T2_3 = Matrix(Ti_j(dh[alpha2], dh[a2], dh[d3], sin(q3), cos(q3)))
T3_4 = Matrix(Ti_j(dh[alpha3], dh[a3], dh[d4], sin(q4), cos(q4)))
T4_5 = Matrix(Ti_j(dh[alpha4], dh[a4], dh[d5], sin(q5), cos(q5)))
T5_6 = Matrix(Ti_j(dh[alpha5], dh[a5], dh[d6], sin(q6), cos(q6)))
T6_EE = Matrix(Ti_j(dh[alpha6], dh[a6], dh[d7], sin(dh[q7]), cos(dh[q7])))
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE


# extract R0_3 from transformation matrices
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3,0:3]


def get_cos_angle(a, b, c):
    """
    Get angle corresponding to side c of a triangle
    using cosine rule
    Inputs:
        a (float)
        b (float) 
        c (float)
    Returns:
        angle (float) # radians
    """
    return acos((a**2 + b**2 - c**2) / (2 * a * b))


def get_shortest_angular_distance_within_limits(from_theta, to_theta, left_limit, right_limit):
    """
    Get angle that corresponds to the shortest angular distance within left and right limits 
    from 'from_theta' angle to 'to_theta' angle
    Inputs:
        from_theta (float) # radians
        to_theta (float)  # radians
        left_limit (float) # radians
        right_limit (float) # radians
    Returns:
        shortest_angular_distance (float) # radians
    """
    shortest_angular_distance = shortest_angular_distance_with_limits(
        from_theta, 
        to_theta, 
        left_limit + from_theta, 
        right_limit + from_theta)[1]
    new_theta = from_theta + shortest_angular_distance
    if left_limit <= new_theta <= right_limit:
        return shortest_angular_distance
    return -np.sign(new_theta) * (radians(360) - abs(shortest_angular_distance))


def handle_calculate_IK(req):
    """
    Calculate joint trajectory using list of Poses (geometry_msgs/Pose)
    Inputs:
        req (server request object)
    Returns:
        CalculateIKResponse(joint_trajectory_list)
    """
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # request initial theta4, theta5, and theta6 from gazebo
        try:
            get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
            previous_theta4 = get_joint_properties('joint_4').position[0]
            previous_theta5 = get_joint_properties('joint_5').position[0]
            previous_theta6 = get_joint_properties('joint_6').position[0]
        except rospy.ServiceException as e:
            rospy.logerr('Get joint properties request failed.')

        # fill joint trajectory list
        print '\n********************' 
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # EE
            EE = Matrix([
                [req.poses[x].position.x],
                [req.poses[x].position.y],
                [req.poses[x].position.z]])
            # roll, pitch, yaw
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([
                req.poses[x].orientation.x,
                req.poses[x].orientation.y,
                req.poses[x].orientation.z,
                req.poses[x].orientation.w])
            # Rrpy
            R0_EE = (rot_axis3(-yaw) * rot_axis2(-pitch) * rot_axis1(-roll)) * \
                    (rot_axis3(-pi) * rot_axis2(pi/2))
            # WC
            WC = EE - (dh[d7]) * R0_EE[:,2]
            side_a = hypot(dh[a3], dh[d4])
            side_b = hypot(hypot(WC[0], WC[1]) - dh[a1], WC[2] - dh[d1])
            side_c = dh[a2]
            angle_a = get_cos_angle(side_b, side_c, side_a)
            angle_b = get_cos_angle(side_a, side_c, side_b)
            angle_c = get_cos_angle(side_a, side_b, side_c)
            # define theta1, theta2, theta3
            theta1 = atan2(WC[1], WC[0])
            theta2 = pi / 2 - angle_a - atan2(WC[2] - dh[d1], hypot(WC[0], WC[1]) - dh[a1])
            theta3 = pi / 2 - angle_b - atan2(-dh[a3], dh[d4])
            # define R3_6
            R0_3_lambda = lambdify([q1, q2, q3], R0_3)
            R3_6 = R0_3_lambda(theta1, theta2, theta3).T * R0_EE
            # joint limits of joint4, joint5, joint6
            upper_joint_limit,lower_joint_limit = radians(350), radians(-350)
            # define theta5
            theta5 = atan2(hypot(R3_6[0,2], R3_6[2,2]), R3_6[1,2])
            theta5_ps_angular_distance = get_shortest_angular_distance_within_limits(
                previous_theta5, theta5, lower_joint_limit, upper_joint_limit)
            theta5_ns_angular_distance = get_shortest_angular_distance_within_limits(
                previous_theta5, -theta5, lower_joint_limit, upper_joint_limit)
            if abs(theta5_ps_angular_distance) < abs(theta5_ns_angular_distance):
                theta5 = previous_theta5 + theta5_ps_angular_distance
            else:
                theta5 = previous_theta5 + theta5_ns_angular_distance
            # define theta4, theta6 (w/ conditional to ensure consistent solution selection)
            if sin(theta5) >= 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            theta4 = previous_theta4 + get_shortest_angular_distance_within_limits(
                previous_theta4, theta4, lower_joint_limit, upper_joint_limit)
            theta6 = previous_theta6 + get_shortest_angular_distance_within_limits(
                previous_theta6, theta6, lower_joint_limit, upper_joint_limit)
            # joint trajectory point
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            previous_theta4, previous_theta5, previous_theta6 = theta4, theta5, theta6
        print '********************'
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    # global FK
    # FK = F_K()
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
