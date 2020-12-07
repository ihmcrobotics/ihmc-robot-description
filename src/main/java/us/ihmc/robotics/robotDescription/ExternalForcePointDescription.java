package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ExternalForcePointDescription extends KinematicPointDescription
{
   public ExternalForcePointDescription(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public ExternalForcePointDescription(ExternalForcePointDescription other)
   {
      super(other);
   }

   @Override
   public ExternalForcePointDescription copy()
   {
      return new ExternalForcePointDescription(this);
   }
}
