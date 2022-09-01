classdef HansCute < handle
  properties(Access = private)
        % Subscribers
       jointStateSub;
       
       % Publisher
       targetJointTrajPub;
       targetJointTrajMsg;

       targetGripperPub;
       targetGripperMsg;
     
  end
  methods(Access = public)
    function self = HansCute()
           [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/hans_cute_robot/target_joint_states');
           [self.targetGripperPub,self.targetGripperMsg] = rospublisher('/hans_cute_robot/target_gripper_state');
    end
    function PublishTargetJoint(self, jointTarget)
           trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
           trajectoryPoint.Positions = jointTarget;
           self.targetJointTrajMsg.Points = trajectoryPoint;
           
           send(self.targetJointTrajPub,self.targetJointTrajMsg);
    end

    function PublishGripperState(self, gripperState)
           self.targetGripperMsg.Data = gripperState;
           send(self.targetGripperPub,self.targetGripperMsg);
    end
  end

end