#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse


class Put(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

    def execute(self, userdata):
        self.loginfo("put: 未実装スキル")
        return "next"
