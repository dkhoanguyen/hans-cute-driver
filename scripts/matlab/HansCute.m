classdef HansCute < handle
  properties(Access = private)
        % Subscribers
       jointStateSub;
       
       % Publisher
       targetJointTrajPub;
       targetJointTrajMsg;
     
  end
  methods(Access = public)
    function self = HansCute()
           [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/hans_cute_robot/target_joint_states');
    end
    function PublishTargetJoint(self, jointTarget)
           trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
           trajectoryPoint.Positions = jointTarget;
           self.targetJointTrajMsg.Points = trajectoryPoint;
           
           send(self.targetJointTrajPub,self.targetJointTrajMsg);
       end
  end

end