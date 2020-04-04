package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PinJointDescription extends OneDoFJointDescription
{

   public PinJointDescription(String name, Vector3D offsetFromParentJoint, Axis3D jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public PinJointDescription(String name, Vector3D offset, Vector3D jointAxis)
   {
      super(name, offset, jointAxis);
   }

}
