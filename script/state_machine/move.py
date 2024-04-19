#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger


class Move(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def exec(self, userdata):
        """部屋に移動する
        """
        # target_room = userdata.room
        # move
        return "next"
