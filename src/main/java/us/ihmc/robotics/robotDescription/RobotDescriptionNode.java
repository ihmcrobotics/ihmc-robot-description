package us.ihmc.robotics.robotDescription;

import java.util.List;

public interface RobotDescriptionNode
{
   public abstract String getName();

   public abstract List<JointDescription> getChildrenJoints();

   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList);

   public RobotDescriptionNode copy();
}
