#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse


class ParseSkill(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.srv_task_parser = rospy.ServiceProxy("/tamhome/task_parser/service", ParseTask)

    def execute(self, userdata):
        # 直前のスキルを保存する
        self.logdebug("save previous skills")
        previous_skill = rospy.get_param("/handyman/commands/next_skill", "start")
        previous_target = rospy.get_param("/handyman/commands/next_skill", "none")

        rospy.set_param("/handyman/commands/previous_skill", previous_skill)
        rospy.set_param("/handyman/commands/previous_target", previous_target)

        # 次に実施すべきタスクを推定
        self.loginfo("parse next skill")
        req = ParseTaskRequest()
        order = rospy.get_param("/handyman/command_text")
        req.order = order
        response: ParseTaskResponse = self.srv_task_parser(req)

        next_skill = response.next_skill
        target = response.target

        # for sopl japanopen 2024
        if target == "rubik's_cube":
            target = "rubick"

        rospy.set_param("/handyman/commands/next_skill", next_skill)
        rospy.set_param("/handyman/commands/next_target", target)

        try:
            return next_skill
        except Exception as e:
            self.logwarn(e)
            self.logwarn(f"invalid skill {next_skill}")
            return "except"
