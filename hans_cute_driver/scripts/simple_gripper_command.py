#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def gripper_comand_action_client():
    rospy.init_node('gripper_command_client')
    client = actionlib.SimpleActionClient('/gripper_command', GripperCommandAction)
    print("Waiting for server to respond")
    client.wait_for_server()

    goal = GripperCommandGoal()
    goal.command.position = 500.0

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    gripper_comand_action_client()