package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class CapsuleDescriptionReadOnly implements ConvexShapeDescriptionReadOnly
{
   private final double radius;
   private final LineSegment3D capToCapLineSegment = new LineSegment3D();

   public CapsuleDescriptionReadOnly(double radius, double height, RigidBodyTransform transformToCenter)
   {
      this(radius, height, Axis3D.Z, transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, LineSegment3D capToCapLineSegment, RigidBodyTransform transformToCenter)
   {
      this.radius = radius;
      this.capToCapLineSegment.set(capToCapLineSegment);
      capToCapLineSegment.applyTransform(transformToCenter);
   }

   public CapsuleDescriptionReadOnly(double radius, double height, Axis3D longAxis, RigidBodyTransform transformToCenter)
   {
      if (height < 2.0 * radius)
         throw new RuntimeException("Capsule height must be at least 2.0 * radius!");
      this.radius = radius;

      switch (longAxis)
      {
         case X:
         {
            capToCapLineSegment.set(-height / 2.0 + radius, 0.0, 0.0, 0.0, 0.0, height / 2.0 - radius);
            break;
         }
         case Y:
         {
            capToCapLineSegment.set(0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius, 0.0);
            break;
         }
         case Z:
         {
            capToCapLineSegment.set(0.0, 0.0, -height / 2.0 + radius, 0.0, 0.0, height / 2.0 - radius);
            break;
         }
      }

      capToCapLineSegment.applyTransform(transformToCenter);
   }

   public double getRadius()
   {
      return radius;
   }

   public void getCapToCapLineSegment(LineSegment3D lineSegmentToPack)
   {
      lineSegmentToPack.set(capToCapLineSegment);
   }

   @Override
   public CapsuleDescriptionReadOnly copy()
   {
      return new CapsuleDescriptionReadOnly(radius, capToCapLineSegment, new RigidBodyTransform());
   }
}
