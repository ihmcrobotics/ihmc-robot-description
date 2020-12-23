package us.ihmc.robotics.robotDescription.joints;

import us.ihmc.euclid.tuple3D.Vector3D;

public class FloatingJointDescription extends JointDescription
{
   private String jointVariableName;

   public FloatingJointDescription()
   {
   }

   public FloatingJointDescription(String name)
   {
      this(name, null);
   }

   public FloatingJointDescription(String name, String jointVariableName)
   {
      super(name, new Vector3D());
      this.jointVariableName = jointVariableName;
   }

   public FloatingJointDescription(FloatingJointDescription other)
   {
      super(other);
      this.jointVariableName = other.jointVariableName;
   }

   public String getJointVariableName()
   {
      return jointVariableName;
   }

   @Override
   public FloatingJointDescription copy()
   {
      return new FloatingJointDescription(this);
   }
}
