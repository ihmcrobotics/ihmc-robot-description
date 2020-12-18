package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;

public class FloatingPlanarJointDescription extends JointDescription
{
   private Plane plane;

   public FloatingPlanarJointDescription(String name, Plane plane)
   {
      super(name, new Vector3D());
      setType(plane);
   }

   public FloatingPlanarJointDescription(FloatingPlanarJointDescription other)
   {
      super(other);
      plane = other.plane;
   }

   public Plane getPlane()
   {
      return plane;
   }

   public void setType(Plane plane)
   {
      this.plane = plane;
   }

   @Override
   public FloatingPlanarJointDescription copy()
   {
      return new FloatingPlanarJointDescription(this);
   }
}
