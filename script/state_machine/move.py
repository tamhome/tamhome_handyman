#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros
import numpy as np

import re  # 文字列の分割に使用
from rapidfuzz.process import extract
from rapidfuzz.process import cdist

from tamlib.utils import Logger
from sigverse_hsrb_nav import HSRBNavigation
from geometry_msgs.msg import Pose2D
from handyman.msg import HandymanMsg


class Move(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

        self.tam_nav = HSRBNavigation()

        self.pub_to_moderator = rospy.Publisher("/handyman/message/to_moderator", HandymanMsg, queue_size=5)

        self.default_room_names = ["kitchen", "living", "lobby", "bed_room"]
        self.navigation_points = {
            "Layout2019HM01": {
                "kitchen": [7.12, 1.42, 1.57],
                "living": [0.83, 3.09, 1.57],
                "lobby": [1.17, -5.74, -1.57],
                "bedroom": [8.74, -6.53, 0],
            },
            "Layout2019HM02": {
                "kitchen": [6.89, 0.71, -1.57],
                "living": [1.82, 11.16, 3.14],
                "lobby": [2.26, 2.05, 3.14],
                "bedroom": [0, 0, 0],
            },

            "Layout2020HM01": {
                "kitchen": [7.93, -1.29, 0],
                # "living": [1.38, 3.37, -1],
                "living": [2.92, -0.199, 1.57],
                "lobby": [0, 0, 0],
                "bedroom": [0.02, 8.23, 1.57],
            },

            "Layout2021HM01": {
                "kitchen": [0, 0, 0],
                "living": [1.19, -2.39, 0],
                "lobby": [-4.53, -8.37, 0],
                "bedroom": [1.61, -7.73, 1.57],
            }
        }

    def rapid_fuzz_word(self, target: str, dictonary: list, th=60) -> str:
        """
        単語ごとの表記ゆれを正す関数

        Args:
            target(str): 対象とする文字列
            dictonary(str): 辞書に登録されている全単語
            th(double): 近さを判定する際のしきい値，その値以下なら置換しない
        """

        # . ! ?も1単語として単語ごとに分割
        target_array = re.findall(r"[\w']+|[.,!?;]", target)
        scores = cdist(target_array, dictonary)

        # 各単語の類似率を表示
        # print(pd.DataFrame(scores, index=target_array, columns=dictonary))
        result_str = ""

        # 類似結果に応じて，置換処理をかける
        for index, score in enumerate(scores):
            if max(score) > th:
                max_index = np.argmax(score)
                # self.loginfo("input: [" + target_array[index] + "] -> replace: [" + dictonary[max_index] + "]")

                # 置換結果の文字列作成
                if (index + 1) < len(scores):
                    # 今の文字が(.,!,?)のいずれかの場合，直前にあるスペースを削除する
                    if dictonary[max_index] in [",", ".", "!", "?"]:
                        result_str = result_str[:-1]
                    result_str += dictonary[max_index] + " "

                # 最後の文字にはスペースを入れない
                else:
                    # 今の文字が(.,!,?)のいずれかの場合，直前にあるスペースを削除する
                    if dictonary[max_index] in [",", ".", "!", "?"]:
                        result_str = result_str[:-1]
                    result_str += dictonary[max_index]

        return result_str

    def execute(self, userdata):
        """部屋に移動する
        """
        room_layout = rospy.get_param("/handyman/room_layout", "Layout2019HM01")
        target_room = rospy.get_param("/handyman/commands/next_target", "kitchen")
        target_room = self.rapid_fuzz_word(target=target_room, dictonary=self.default_room_names)

        self.loginfo(f"target layout is {room_layout}")
        self.loginfo(f"target room is {target_room}")

        goal_pose2d = Pose2D()
        try:
            goal_pose = self.navigation_points[room_layout][target_room]
            goal_pose2d.x = goal_pose[0]
            goal_pose2d.y = goal_pose[1]
            goal_pose2d.theta = goal_pose[2]
        except Exception as e:
            self.logwarn(e)
            goal_pose2d.x = 0
            goal_pose2d.y = 0
            goal_pose2d.theta = 0

        self.tam_nav.navigation(goal_pose=goal_pose2d)

        msg = HandymanMsg()
        msg.message = "Room_reached"
        msg.detail = "Room_reached"
        self.pub_to_moderator.publish(msg)

        return "next"
