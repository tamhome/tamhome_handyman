#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros
from typing import List

from tamlib.utils import Logger
from tamhome_skills import Grasp, Find

from geometry_msgs.msg import Pose
from handyman.msg import HandymanMsg


class GraspState(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.tam_grasp = Grasp()
        self.tam_find = Find()

        # ros interface
        self.pub_to_moderator = rospy.Publisher("/handyman/message/to_moderator", HandymanMsg, queue_size=5)

    def execute(self, userdata):
        """物体の把持を行う
        """
        target_pose_list: List = rospy.get_param("/tamhome_skills/object_detection/current_pose_odom", "none")
        target_object_name = rospy.get_param("/handyman/commands/next_target", "object")

        # target_poseが指定されていない場合は，適当な物体を見つける
        if target_pose_list == "none":
            self.tam_find.find_obj(target_object_name)

        # 把持姿勢を与えて，把持を行う
        target_pose = Pose()
        target_pose.position.x = target_pose_list[0]
        target_pose.position.y = target_pose_list[1]
        target_pose.position.z = target_pose_list[2]

        target_pose.orientation.x = target_pose_list[3]
        target_pose.orientation.y = target_pose_list[4]
        target_pose.orientation.z = target_pose_list[5]
        target_pose.orientation.w = target_pose_list[6]

        status = self.tam_grasp.grasp_obj_by_pose(target_pose, timeout=30, source_frame="odom")

        if status is True:
            # 把持したため値をリセット
            rospy.set_param("/tamhome_skills/object_detection/current_pose_odom", "none")

            if status is True:
                msg = HandymanMsg()
                msg.message = "Object_grasped"
                msg.detail = "Object_grasped"
                self.pub_to_moderator.publish(msg)

            try:
                msg = rospy.wait_for_message("/handyman/message/to_robot", HandymanMsg, timeout=1)
                if msg.message == "Task_failed":
                    return "except"
                else:
                    self.logsuccess("I can grasp correct object!")
            except Exception as e:
                self.logwarn(e)
                pass

            return "next"

        else:
            # TODO: 把持を再トライするように修正
            # 把持したため値をリセット
            rospy.set_param("/tamhome_skills/object_detection/current_pose_odom", "none")

            return "except"
