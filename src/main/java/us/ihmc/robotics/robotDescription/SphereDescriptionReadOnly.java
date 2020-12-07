package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class SphereDescriptionReadOnly implements ConvexShapeDescriptionReadOnly
{
   private final double radius;
   private final RigidBodyTransform rigidBodyTransform;

   public SphereDescriptionReadOnly(double radius, RigidBodyTransformReadOnly rigidBodyTransform)
   {
      this.radius = radius;
      this.rigidBodyTransform = new RigidBodyTransform(rigidBodyTransform);
   }

   public double getRadius()
   {
      return radius;
   }

   public void getRigidBodyTransform(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.set(rigidBodyTransform);
   }

   @Override
   public SphereDescriptionReadOnly copy()
   {
      return new SphereDescriptionReadOnly(radius, rigidBodyTransform);
   }
}
