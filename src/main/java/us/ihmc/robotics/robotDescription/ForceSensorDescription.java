package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class ForceSensorDescription extends SensorDescription
{
   private boolean useGroundContactPoints = true;

   private boolean useShapeCollision = false;

   public ForceSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

   public ForceSensorDescription(ForceSensorDescription other)
   {
      super(other);
      useGroundContactPoints = other.useGroundContactPoints;
      useShapeCollision = other.useShapeCollision;
   }

   public boolean useShapeCollision()
   {
      return useShapeCollision;
   }

   public void setUseShapeCollision(boolean useShapeCollision)
   {
      this.useShapeCollision = useShapeCollision;
   }

   public boolean useGroundContactPoints()
   {
      return useGroundContactPoints;
   }

   public void setUseGroundContactPoints(boolean useGroundContactPoints)
   {
      this.useGroundContactPoints = useGroundContactPoints;
   }

   @Override
   public ForceSensorDescription copy()
   {
      return new ForceSensorDescription(this);
   }
}
