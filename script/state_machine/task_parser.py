#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger


class TaskParser(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def exec(self, userdata):
        """サービスにアクセスし，次に実行するタスクを選択
        """
        next_action = "move"
        return next_action
