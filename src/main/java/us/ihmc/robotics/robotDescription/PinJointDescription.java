package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PinJointDescription extends OneDoFJointDescription
{
   public PinJointDescription(String name, Tuple3DReadOnly offsetFromParentJoint, Vector3DReadOnly jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public PinJointDescription(PinJointDescription other)
   {
      super(other);
   }

   @Override
   public PinJointDescription copy()
   {
      return new PinJointDescription(this);
   }
}
