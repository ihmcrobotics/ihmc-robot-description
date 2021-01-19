package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class BallAndSocketJointDescription extends JointDescription
{

   public BallAndSocketJointDescription(String name, Tuple3DReadOnly offsetFromParentJoint)
   {
      super(name, offsetFromParentJoint);
   }

}
