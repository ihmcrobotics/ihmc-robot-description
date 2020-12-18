package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class KinematicPointDescription
{
   private String name;
   private Vector3D offsetFromJoint = new Vector3D();

   public KinematicPointDescription(String name, Tuple3DReadOnly offsetFromJoint)
   {
      setName(name);
      this.offsetFromJoint.set(offsetFromJoint);
   }

   public KinematicPointDescription(KinematicPointDescription other)
   {
      name = other.name;
      offsetFromJoint.set(other.offsetFromJoint);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void getOffsetFromJoint(Tuple3DBasics offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public Vector3D getOffsetFromJoint()
   {
      return offsetFromJoint;
   }

   public void setOffsetFromJoint(Tuple3DReadOnly offsetFromJoint)
   {
      this.offsetFromJoint.set(offsetFromJoint);
   }

   public KinematicPointDescription copy()
   {
      return new KinematicPointDescription(this);
   }
}
