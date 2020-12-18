package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SensorDescription
{
   private String name;
   private final RigidBodyTransform transformToJoint = new RigidBodyTransform();

   public SensorDescription(String name, Tuple3DReadOnly offsetFromJoint)
   {
      this.name = name;
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public SensorDescription(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      this.name = name;
      this.transformToJoint.set(transformToJoint);
   }

   public SensorDescription(SensorDescription other)
   {
      name = other.name;
      transformToJoint.set(other.transformToJoint);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setOffsetFromJoint(Tuple3DReadOnly offsetFromJoint)
   {
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public void getTransformToJoint(RigidBodyTransformBasics transformToJointToPack)
   {
      transformToJointToPack.set(transformToJoint);
   }

   public void setTransformToJoint(RigidBodyTransformReadOnly transformToJoint)
   {
      this.transformToJoint.set(transformToJoint);
   }

   public RigidBodyTransform getTransformToJoint()
   {
      return transformToJoint;
   }

   public Vector3DBasics getOffsetFromJoint()
   {
      return transformToJoint.getTranslation();
   }

   public SensorDescription copy()
   {
      return new SensorDescription(this);
   }
}
