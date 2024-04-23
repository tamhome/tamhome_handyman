#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse
from tamhome_skills import Find


class FindState(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.tam_find = Find()

    def execute(self, userdata):
        target_object_name = rospy.get_param("/handyman/commands/next_target", "object")
        self.loginfo(f"searching: {target_object_name}")
        self.tam_find.find_obj(target_object_name, is_rotation=True, confidence_th=0.85)
        return "next"
