#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import roslib
from tamlib.utils import Logger
from std_srvs.srv import SetBool, SetBoolRequest

from handyman.msg import HandymanMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import LoadMap, LoadMapRequest, LoadMapResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap, SetMapRequest, SetMapResponse


class Init(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def execute(self, userdata):
        return "next"


class Wait4Start(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self, loglevel="INFO")

        self.nav_pkd_dir = roslib.packages.get_pkg_dir("sigverse_hsrb_nav")
        self.map_version = "orig"

        # ros interface
        self.pub_to_moderator = rospy.Publisher("/handyman/message/to_moderator", HandymanMsg, queue_size=5)
        self.inital_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        self.srv_change_map = rospy.ServiceProxy("/change_map", LoadMap)
        self.srv_set_map = rospy.ServiceProxy("/set_map", SetMap)

        # タスク理解用プロンプトのリセット
        self.srv_reset_prompt = rospy.ServiceProxy("/tamhome/task_parser/reset_prompt", Empty)

    def set_inital_pose(self):
        self.loginfo("set inital pose.")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 1
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        for _ in range(5):
            self.inital_pose.publish(msg)
            rospy.sleep(0.5)

    def wait_for_ready_msg(self, ready_msg="Are_you_ready?") -> None:
        """ready_for_messageの送信を待つ関数
        Args:
            ready_msg(string): 開始の際に送信される特定のメッセージを指定
            defaults to Are_you_ready?
        """
        self.loginfo(f"wait for ready message: {ready_msg}")
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/handyman/message/to_robot", HandymanMsg, timeout=None)
            self.loginfo(msg.message)
            if msg.message == ready_msg:
                self.logsuccess(f"Start task {ready_msg}")
                msg = HandymanMsg()
                msg.message = "I_am_ready"
                self.pub_to_moderator.publish(msg)
                break

    def get_command(self, instruction_msg="Instruction") -> str:
        """handymanからのコマンド送信を待ち，rosparamに設定する
        """
        self.loginfo(f"waiting {instruction_msg}")
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/handyman/message/to_robot", HandymanMsg, timeout=None)
            if msg.message == instruction_msg:
                command = msg.detail
                self.loginfo(f"Instruction command: {msg.detail}")
                rospy.set_param("/handyman/command_text", command)
                break

        return command

    def map_detector(self, environment_msg="Environment") -> str:
        """Environmentで送られてくる部屋のレイアウト情報を取得する
        """
        self.loginfo(f"waiting {environment_msg}")
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/handyman/message/to_robot", HandymanMsg, timeout=None)
            if msg.message == environment_msg:
                map_name = msg.detail
                self.loginfo(f"Using environment name: {map_name}")
                rospy.set_param("/handyman/room_layout", map_name)

                # マップの切り替え
                change_map_req = LoadMapRequest()
                change_map_req.map_url = f"{self.nav_pkd_dir}/map/handyman/{self.map_version}/{map_name}.yaml"
                response = self.srv_change_map(change_map_req)
                if response.result == 255:
                    self.logwarn(f"指定されたマップがありませんでした: {change_map_req.map_url}")
                elif response.result == 0:
                    self.logsuccess(f"指定されたマップに切り替えました: {change_map_req.map_url}")
                else:
                    self.loginfo(f"不明なレスポンスがありました．Rvizを確認してください．: {response.result}")
                rospy.sleep(2)

                # amclがもつ地図データを更新
                new_map_msg: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid)
                set_map_req = SetMapRequest()
                set_map_req.map = new_map_msg

                # ヘッダの管理
                set_map_req.initial_pose.header.stamp = rospy.Time.now()
                set_map_req.initial_pose.header.frame_id = "map"

                set_map_req.initial_pose.pose.pose.position.x = 0
                set_map_req.initial_pose.pose.pose.position.y = 0
                set_map_req.initial_pose.pose.pose.position.z = 0
                set_map_req.initial_pose.pose.pose.orientation.x = 0
                set_map_req.initial_pose.pose.pose.orientation.y = 0
                set_map_req.initial_pose.pose.pose.orientation.z = 0
                set_map_req.initial_pose.pose.pose.orientation.w = 1
                set_map_req.initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

                self.srv_set_map(set_map_req)
                break

        return map_name

    def execute(self, userdata):
        # プロンプトのリセット
        req = EmptyRequest()
        self.srv_reset_prompt(req)

        # mapの切り替え
        self.map_detector()
        self.set_inital_pose()

        # wait for ready
        is_wait_ready = rospy.get_param("/handyman/wait_to_ready", True)
        if is_wait_ready:
            self.wait_for_ready_msg()
        self.loginfo("I'm preparing to start.")

        # instruction commandの獲得
        self.get_command()
        return "next"


class Start(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)

    def execute(self, userdata):
        self.loginfo("Start")
        return "next"


class Finish(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)
        self.pub_to_moderator = rospy.Publisher("/handyman/message/to_moderator", HandymanMsg, queue_size=5)

    def execute(self, userdata):
        msg = HandymanMsg()
        msg.message = "Task_finished"
        msg.detail = "Task_finished"
        self.pub_to_moderator.publish(msg)

        msg = HandymanMsg()
        msg.message = "Give_up"
        msg.detail = "Give_up"
        self.pub_to_moderator.publish(msg)

        self.loginfo("Finished")

        return "finish"


class Except(smach.State, Logger):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        Logger.__init__(self)
        self.pub_to_moderator = rospy.Publisher("/handyman/message/to_moderator", HandymanMsg, queue_size=5)

    def execute(self, userdata):
        self.logerr("Excepted")
        msg = HandymanMsg()
        msg.message = "Give_up"
        msg.detail = "Give_up"
        self.pub_to_moderator.publish(msg)

        try:
            return "recovery"
        except Exception as e:
            self.logwarn(e)
            return "except"
