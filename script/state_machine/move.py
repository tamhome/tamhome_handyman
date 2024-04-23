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

        self.default_room_names = ["kitchen", "living", "lobby", "bedroom"]
        self.navigation_points = {
            "Layout2019HM01": {
                "kitchen": {
                    "root": [7.12, 1.42, 1.57],
                    "wooden_side_table": [4.73, 2.20, 1.57],
                    "dining_table": [7.12, 4.77, -1.57],
                    "custom_kitchen": [8.2, 4.77, 0],
                    "custom_kitchen#1": [8.2, 4.77, 0],
                    "custom_kitchen#2": [8.2, 4.77, 0],
                    "sink": [8.2, 4.77, 0],
                    "blue_cupboard": [8.79, 3.00, 0],
                },
                "living": {
                    "root": [0.83, 3.09, 1.57],
                    "white_side_table": [2.02, 0.36, -1.57],
                    "sofa": [0.0, 4.49, 3.14],
                    "square_low_table": [2.10, 3.76, 3.14],
                    "TV_rack": [2.29, 5.81, 3.14],
                    "trash_box": [-0.38, 1.21, 3.14],

                },
                "lobby": {
                    "root": [1.17, -5.74, -1.57],
                    "white_side_table": [1.17, -5.74, -1.57],
                    "corner_sofa": [0, -5.64, -1.57],
                    "sofa": [0, -4.58, 3.14],
                    "white_rack": [0, -2.93, 3.14],
                    "trash_box": [-0.28, -2.03, 3.14],
                    "armchair": [1.98, -3.45, 1.57],

                },
                "bedroom": {
                    "root": [8.74, -6.53, 0],
                    "wooden_bed": [7.19, -4.86, 1.57],
                    "white_side_table": [9.35, -3.40, 1.57],
                    "white_side_table_1": [8.88, -6.32, 0],
                    "white_side_table_2": [9.35, -3.40, 1.57],
                    "cardboard_box": [8.70, -4.80, 0],
                    "wagon": [8.23, -3.20, 1.57],
                },
            },
            "Layout2019HM02": {
                "kitchen": {
                    "root": [6.89, 0.71, -1.57],
                    "trashbox_c01": [6.81, 3.94, -1.57],
                    "dining_table": [6.90, 0.65, -1.57],
                    "wooden_cupboard": [8.49, -3.12, 0],
                    "blue_cupboard": [8.25, -1.43, 0],
                    "sink": [8.11, 0.90, 0],
                    "wooden_side_table": [8.12, 3.33, 0],
                },
                "living": {
                    "root": [1.82, 11.16, 3.14],
                    "white_side_table": [0.80, 8.02, -1.57],
                    "sofa": [-0.50, 9.28, 3.14],
                    "round_low_table": [2.60, 9.08, 3.14],
                    "square_low_table": [2.10, 11.16, 3.14],
                    "TV_rack#2": [-0.29, 10.34, 2.9],
                    "white_chair": [4.59, 11.71, 0],
                    "cardboard_box": [2.23, 8.22, -1.57]
                },
                "lobby": {
                    "root": [2.26, 2.05, 3.14],
                    "dining_table": [2.12, 1.74, 3.14],
                    "white_side_table": [1.11, -0.37, -1.57],
                    "white_shelf": [-0.48, -0.37, -1.57],
                    "sofa": [-0.48, 1.89, 3.14],
                    "wagon": [2.71, 0.32, 0],
                    "wagon_c01#1": [2.71, 0.32, 0],
                    "wagon_c01#2": [-0.37, 4.33, 3.14],
                    "armchair": [-0.37, 4.33, 3.14],
                    "wooden_side_table": [0.69, 4.80, 1.57],
                    "wooden_shelf": [1.78, 4.80, 1.57],
                },
                "bedroom": {
                    "root": [0, 0, 0],
                },
            },

            "Layout2020HM01": {
                "kitchen": {
                    "root": [7.93, -1.29, 0],
                    "wooden_cupboard": [8.20, -1.36, 0],
                    "blue_cupboard": [8.20, 0.30, 0],
                    "custome_kitchen": [7.23, 1.23, 3.14],
                },
                "living": {
                    "root": [2.92, -0.309, 1.57],
                    "shelf_A": [1.38, 3.37, -1],
                    "wooden_side_table": [2.92, -0.199, -1.57],
                    "square_low_table": [2.70, -0.199, -1.57],
                    "white_side_table": [-0.29, 3.65, 3.14],
                    "wooden_shelf": [2.33, 4.45, 1.57]
                },
                "lobby": {
                    "root": [0, 0, 0],
                },
                "bedroom": {
                    "root": [0.02, 8.23, 1.57],
                    "iron_bed": [3.19, 7.96, 0],
                    "white_side_table": [2.88, 7.43, -1.57],
                    "round_law_table": [0, 8.07, -1.57],
                    "cardboard_box": [1.18, 8.07, -1.57],
                    "changing_table": [2.94, 8.07, -1.57],
                },
            },

            "Layout2021HM01": {
                "kitchen": {
                    "root": [0, 0, 0],
                },
                "living": {
                    "root": [2.18, -0.30, -1.57],
                    "white_shelf": [4.05, 1.15, 0],
                    "white_side_table": [2.05, 0.36, 1.57],
                    "dining_table": [2.18, -0.66, -1.57],
                    "TV_rack": [3.53, -2.39, 0],
                    "wagon": [-1.18, -2.39, 3.14],
                    "trash_box": [-1.18, -1.02, 3.14],
                },
                "lobby": {
                    "root": [-4.53, -8.37, 0],
                    "sofa": [-4.53, -8.37, 0],
                    "cardboard_box": [-5.64, -7.08, 3.14],
                    "white_side_table": [-3.94, -9.85, -1.57],
                    "dining_table": [-4.54, -8.41, 0],
                    "trash_box": [-3.18, -5.12, 1.57],
                    "wooden_shelf": [-4.16, -5.31, 1.57]

                },
                "bedroom": {
                    "root": [1.61, -7.73, 1.57],
                    "white_side_table": [1.55, -7.54, 1.57],
                    "magazine_rack_B": [0.80, -7.98, 3.14],
                    "wagon": [0.80, -9.00, 3.14],
                    "wooden_bed": [2.58, -10.61, 3.14],
                    "armchair": [4.30, -9.14, -1.57],
                },
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
        target_room = self.rapid_fuzz_word(target=target_room, dictonary=self.default_room_names, th=90)

        goal_pose2d = Pose2D()

        # 家具への移動
        if target_room == "":
            target_furniture = rospy.get_param("/handyman/commands/next_target", "kitchen")
            target_room = rospy.get_param("/handyman/commands/previous_target", "kitchen")

            self.loginfo(f"target layout is {room_layout}")
            self.loginfo(f"target room is {target_room}")
            self.loginfo(f"target furniture is {target_furniture}")

            try:
                goal_pose = self.navigation_points[room_layout][target_room][target_furniture]

            except Exception as e:
                self.logwarn(e)
                try:
                    goal_pose = self.navigation_points[room_layout][target_room]["root"]
                except Exception as e:
                    self.logwarn(e)
                    goal_pose = self.navigation_points[room_layout]["living"]["root"]

            try:
                goal_pose2d.x = goal_pose[0]
                goal_pose2d.y = goal_pose[1]
                goal_pose2d.theta = goal_pose[2]
            except Exception as e:
                self.logwarn(e)
                return "except"

        # 部屋への移動
        else:

            self.loginfo(f"target layout is {room_layout}")
            self.loginfo(f"target room is {target_room}")

            try:
                goal_pose = self.navigation_points[room_layout][target_room]["root"]
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
