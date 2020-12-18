package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ConvexPolytopeDescriptionReadOnly implements ConvexShapeDescriptionReadOnly
{
   //TODO: Trying to create a read only version by copying a full version. Should be a more efficient way to do this...
   private final ConvexPolytope3D convexPolytope;

   public ConvexPolytopeDescriptionReadOnly(ConvexPolytope3DReadOnly polytope, RigidBodyTransform rigidBodyTransform)
   {
      convexPolytope = new ConvexPolytope3D(polytope);
      convexPolytope.applyTransform(rigidBodyTransform);
   }

   public ConvexPolytope3D getConvexPolytope()
   {
      return new ConvexPolytope3D(convexPolytope);
   }

   @Override
   public ConvexPolytopeDescriptionReadOnly copy()
   {
      return new ConvexPolytopeDescriptionReadOnly(convexPolytope, new RigidBodyTransform());
   }
}
