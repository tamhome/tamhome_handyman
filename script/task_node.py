#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import smach
import smach_ros
# from hsrlib.hsrif import HSRInterfaces

from state_machine import standard
from state_machine import parse_skill, move, grasp, find, deliver, put


class InteractiveCleanupStateMachine():
    def __init__(self) -> None:
        """
        必要なモジュールを初期化
        """
        self.start_state = rospy.get_param(rospy.get_name() + "/start_state", "Start")

        # ステートマシンの宣言
        self.sm = smach.StateMachine(outcomes=["exit"])

        with self.sm:
            smach.StateMachine.add(
                "Init",
                standard.Init(["next", "except"]),
                transitions={"next": "Wait4Start", "except": "Except"},
            )
            smach.StateMachine.add(
                "Wait4Start",
                standard.Wait4Start(["next", "except"]),
                transitions={"next": self.start_state, "except": "Except"},
            )
            smach.StateMachine.add(
                "Start",
                standard.Start(["next", "except"]),
                transitions={"next": "ParseSkill", "except": "Except"},
            )

            # Interactive cleanup
            smach.StateMachine.add(
                "ParseSkill",
                parse_skill.ParseSkill(["move", "find", "grasp", "deliver", "put", "except", "done"]),
                transitions={
                    "move": "Move",
                    "find": "Find",
                    "grasp": "Grasp",
                    "deliver": "Deliver",
                    "put": "Put",
                    "done": "Finish",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Move",
                move.Move(["next", "loop", "except"]),
                transitions={
                    "next": "ParseSkill",
                    "loop": "Move",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Find",
                find.FindState(["next", "loop", "except"]),
                transitions={
                    "next": "ParseSkill",
                    "loop": "Find",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Grasp",
                grasp.GraspState(["next", "loop", "except"]),
                transitions={
                    "next": "ParseSkill",
                    "loop": "Grasp",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Deliver",
                deliver.DeliverState(["next", "loop", "except"]),
                transitions={
                    "next": "ParseSkill",
                    "loop": "Deliver",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Put",
                put.PutState(["next", "loop", "except"]),
                transitions={
                    "next": "ParseSkill",
                    "loop": "Put",
                    "except": "Except",
                },
            )
            smach.StateMachine.add(
                "Finish",
                standard.Finish(["finish"]),
                transitions={"finish": "Init"},
            )
            smach.StateMachine.add(
                "Except",
                standard.Except(["except", "recovery"]),
                transitions={
                    "except": "exit",
                    "recovery": "Init"
                },
            )

    def delete(self) -> None:
        del self.sm

    def run(self) -> None:
        self.sm.execute()


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    cls = InteractiveCleanupStateMachine()
    rospy.on_shutdown(cls.delete)
    try:
        cls.run()
    except rospy.exceptions.ROSException as e:
        rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        rospy.logerr("[" + rospy.get_name() + "]: " + str(e))


if __name__ == "__main__":
    main()
