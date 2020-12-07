package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class SliderJointDescription extends OneDoFJointDescription
{
   public SliderJointDescription(String name, Tuple3DReadOnly offsetFromParentJoint, Vector3DReadOnly jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public SliderJointDescription(SliderJointDescription other)
   {
      super(other);
   }

   @Override
   public SliderJointDescription copy()
   {
      return new SliderJointDescription(this);
   }
}
