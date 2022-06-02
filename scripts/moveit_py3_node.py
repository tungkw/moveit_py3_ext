#!/usr/bin/env python2
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import moveit_commander
import geometry_msgs.msg

import moveit_py3.srv
from config import config as cfg


def set_collision():

    # planning scene collision
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0-0.1/2
    box_size = (0.48, 0.58, 0.1)
    box_name = 'sit'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.7 + 0.1/2
    box_size = (1, 1, 0.1)
    box_name = 'ceiling'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = -0.48/2-0.1/2
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.5
    box_size = (0.1, 1, 1)
    box_name = 'back_wall'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = 0.8 + 0.1/2
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.5
    box_size = (0.1, 1, 1)
    box_name = 'front_wall'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0.58/2 + 0.1/2
    box_pose.pose.position.z = 0.5
    box_size = (1, 0.1, 1)
    box_name = 'left_wall'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = 0.48/2 + 0.2
    box_pose.pose.position.y = 0.58/2 + 0.2
    box_pose.pose.position.z = 0.5
    box_size = (0.4, 0.4, 1)
    box_name = 'right_corner'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)

    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.05-0.1/2
    box_size = (2, 2, 0.1)
    box_name = 'grasp_plane'
    while not box_name in scene.get_known_object_names():
        scene.add_box(box_name, box_pose, size=box_size)




if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(cfg.package_name, anonymous=True)
    rate = rospy.Rate(125)

    # set_collision()

    # ========== service ==========
    # =============================

    # moveit commander
        # func_set_pose(target_pose, True)
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_end_effector_link("tool0")
    # move_group.set_end_effector_link("ee_link")
    move_group.set_pose_reference_frame("world")
    # move_group.set_max_velocity_scaling_factor(0.2)
    # move_group.set_max_acceleration_scaling_factor(0.2)

    jac = np.array(move_group.get_jacobian_matrix(move_group.get_current_joint_values()))

    def set_pose(req):
        move_group.set_pose_target(req.target_pose)
        move_group.go(wait=req.wait)
        return moveit_py3.srv.SetPoseResponse()
    rospy.Service(cfg.set_pose_srv_name, moveit_py3.srv.SetPose, set_pose)

    def get_pose(req):
        pose = move_group.get_current_pose("tool0")
        res = moveit_py3.srv.GetPoseResponse()
        res.pose = pose.pose
        return res
    rospy.Service(cfg.get_pose_srv_name, moveit_py3.srv.GetPose, get_pose)

    def set_positions(req):
        plan = move_group.go(req.target_positions, req.wait)
        return moveit_py3.srv.SetPositionsResponse()
    rospy.Service(cfg.set_positions_srv_name, moveit_py3.srv.SetPositions, set_positions)

    def get_positions(req):
        res = moveit_py3.srv.GetPositionsResponse()
        res.positions = move_group.get_current_joint_values()
        return res
    rospy.Service(cfg.get_positions_srv_name, moveit_py3.srv.GetPositions, get_positions)

    rospy.loginfo("moveit_py3 service ready.")

    rospy.spin()
