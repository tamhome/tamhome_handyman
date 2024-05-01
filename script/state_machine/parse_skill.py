#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros

from tamlib.utils import Logger
from tamhome_task_parser.srv import ParseTask, ParseTaskRequest, ParseTaskResponse
from handyman.msg import HandymanMsg


class ParseSkill(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.srv_task_parser = rospy.ServiceProxy("/tamhome/task_parser/service", ParseTask)

    def execute(self, userdata):
        # 直前のスキルを保存する

        # このセッションがすでに終了していないかどうかを検証する
        for _ in range(3):
            try:
                msg = rospy.wait_for_message("/handyman/message/to_robot", HandymanMsg, timeout=0.5)
                if msg.message == "Are_you_ready?" or msg.message == "Environment":
                    # 直前のセッションが終了しているということであるため，exceptからInitに戻る
                    self.loginfo("セッション終了: 次のセッション開始の準備を行います．")
                    return "except"
            except Exception as e:
                # メッセージが飛んでいない = セッションが続いている
                self.logtrace(e)
                pass

        self.logdebug("save previous skills")
        previous_skill = rospy.get_param("/handyman/commands/next_skill", "start")
        previous_target = rospy.get_param("/handyman/commands/next_target", "none")

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

        rospy.set_param("/handyman/commands/next_skill", next_skill)
        rospy.set_param("/handyman/commands/next_target", target)

        try:
            return next_skill
        except Exception as e:
            self.logwarn(e)
            self.logwarn(f"invalid skill {next_skill}")
            return "except"
