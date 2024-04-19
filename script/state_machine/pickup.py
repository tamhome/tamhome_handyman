#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger


class PickUp(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def exec(self, userdata):
        """物体の探索と把持を行う
        """
        # 物体の探索
        target_frame = "target_frame"
        return "next"
