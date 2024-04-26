#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse
from tamhome_skills import Put, Grasp
from sigverse_hsrlib import MoveJoints


class PutState(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.tam_put = Put()
        self.tam_grasp = Grasp()
        self.tam_move_joints = MoveJoints()

        self.put_joints = {
            "default": [0.5, -0.9, 0.0, -0.65, 0.0],
            "cardboard_box": [0.3, -0.9, 0.0, -0.65, 0.0],
            "trash_box": [0.6, -0.9, 0.0, -0.65, 0.0],
            "armchair": [0.4, -0.9, 0.0, -0.65, 0.0],
        }

    def execute(self, userdata):
        target_furniture = rospy.get_param("/handyman/commands/next_target", "cardboard_box")
        self.tam_move_joints.move_head(0, -0.2)
        rospy.sleep(1.5)

        self.loginfo(f"target furniture is {target_furniture}")

        try:
            res = self.tam_put.put_for_furniture(target_furniture)
            res = False
            if res is False:
                try:
                    target_joint = self.put_joints[target_furniture]
                except Exception as e:
                    self.logwarn("指定の家具に対する姿勢はありません．")
                    target_joint = self.put_joints["default"]

                try:
                    self.tam_move_joints.move_arm_by_pose(
                        target_joint[0],
                        target_joint[1],
                        target_joint[2],
                        target_joint[3],
                        target_joint[4],
                    )
                    rospy.sleep(3)
                    self.tam_grasp._move_forward(0.3)
                    rospy.sleep(1)
                    self.tam_move_joints.gripper(3.14)
                except Exception as e:
                    self.logwarn(e)

            return "next"
        except Exception as e:
            self.logwarn(e)
            return "except"
