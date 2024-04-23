#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse
from tamhome_skills import Deliver


class DeliverState(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.tam_deliver = Deliver()

    def execute(self, userdata):
        self.loginfo("pass to nearest person")
        self.tam_deliver.pass_to_near_person(self)
        return "next"
