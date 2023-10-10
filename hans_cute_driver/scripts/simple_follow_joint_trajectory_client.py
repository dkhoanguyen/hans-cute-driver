#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# Callback function for the joint state subscriber
def joint_state_callback(msg):
    global current_joint_positions
    current_joint_positions = msg.position

def follow_joint_trajectory_client():
    rospy.init_node('follow_joint_trajectory_client')
    client = actionlib.SimpleActionClient('/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server to respond")
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ['shoulder_1', 'shoulder_2', 'elbow_1', 'elbow_2', 'wrist_1', 'wrist_2', 'wrist_3']  # Replace with your joint names

    while not rospy.is_shutdown():
        if current_joint_positions:
            print("Sending goal")
            # Create a point using the current joint positions
            point = JointTrajectoryPoint()
            point.positions = current_joint_positions  # Use the current joint positions
            point.time_from_start = rospy.Duration(0)  # Start immediately

            goal.trajectory.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [0,0,0,0,0,0,0]  # Use the current joint positions
            point.time_from_start = rospy.Duration(5)  # Start after 5s
            goal.trajectory.points.append(point)

            client.send_goal(goal)
            client.wait_for_result()
            break
    print(client.get_result())
    
if __name__ == '__main__':
    current_joint_positions = None  # Initialize as None
    try:
        rospy.Subscriber('/joint_states', JointState, joint_state_callback)
        print("Waiting for joint states")
        while current_joint_positions is None and rospy.is_shutdown():
            print("Waiting for joint state information...")
            rospy.sleep(1.0)

        follow_joint_trajectory_client()
    except rospy.ROSInterruptException:
        pass
