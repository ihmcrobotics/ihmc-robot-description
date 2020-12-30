package us.ihmc.robotics.robotDescription.joints;

import java.util.List;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class OneDoFJointDescription extends JointDescription
{
   private double positionLowerLimit = Double.NEGATIVE_INFINITY, positionUpperLimit = Double.POSITIVE_INFINITY;
   private double kpPositionLimit, kdPositionLimit;

   private double velocityLowerLimit = Double.NEGATIVE_INFINITY, velocityUpperLimit = Double.POSITIVE_INFINITY;
   private double kpVelocityLimit;

   private double effortLowerLimit = Double.NEGATIVE_INFINITY, effortUpperLimit = Double.POSITIVE_INFINITY;

   private double damping;
   private double stiction;

   private final Vector3D axis = new Vector3D();

   public OneDoFJointDescription(String name)
   {
      super(name);
   }

   public OneDoFJointDescription(String name, Tuple3DReadOnly offset, Vector3DReadOnly axis)
   {
      super(name, offset);
      this.axis.set(axis);
   }

   public OneDoFJointDescription(OneDoFJointDescription other)
   {
      super(other);
      axis.set(other.axis);

      positionLowerLimit = other.positionLowerLimit;
      positionUpperLimit = other.positionUpperLimit;
      kpPositionLimit = other.kpPositionLimit;
      kdPositionLimit = other.kdPositionLimit;
      velocityLowerLimit = other.velocityLowerLimit;
      velocityUpperLimit = other.velocityUpperLimit;
      kpVelocityLimit = other.kpVelocityLimit;
      effortLowerLimit = other.effortLowerLimit;
      effortUpperLimit = other.effortUpperLimit;
      damping = other.damping;
      stiction = other.stiction;
   }

   public void setPositionLowerLimit(double positionLowerLimit)
   {
      this.positionLowerLimit = positionLowerLimit;
   }

   public void setPositionUpperLimit(double positionUpperLimit)
   {
      this.positionUpperLimit = positionUpperLimit;
   }

   public void setPositionLimits(double positionLowerLimit, double positionUpperLimit)
   {
      this.positionLowerLimit = positionLowerLimit;
      this.positionUpperLimit = positionUpperLimit;
   }

   public void setPositionLimits(double positionLimit)
   {
      this.positionLowerLimit = -positionLimit;
      this.positionUpperLimit = +positionLimit;
   }

   public void setKpPositionLimit(double kpPositionLimit)
   {
      this.kpPositionLimit = kpPositionLimit;
   }

   public void setKdPositionLimit(double kdPositionLimit)
   {
      this.kdPositionLimit = kdPositionLimit;
   }

   public void setPositionLimitGains(double kpPositionLimit, double kdPositionLimit)
   {
      this.kpPositionLimit = kpPositionLimit;
      this.kdPositionLimit = kdPositionLimit;
   }

   public void setVelocityLowerLimit(double velocityLowerLimit)
   {
      this.velocityLowerLimit = velocityLowerLimit;
   }

   public void setVelocityUpperLimit(double velocityUpperLimit)
   {
      this.velocityUpperLimit = velocityUpperLimit;
   }

   public void setVelocityLimits(double velocityLowerLimit, double velocityUpperLimit)
   {
      this.velocityLowerLimit = velocityLowerLimit;
      this.velocityUpperLimit = velocityUpperLimit;
   }

   public void setVelocityLimits(double velocityLimit)
   {
      this.velocityLowerLimit = -velocityLimit;
      this.velocityUpperLimit = +velocityLimit;
   }

   public void setKpVelocityLimit(double kpVelocityLimit)
   {
      this.kpVelocityLimit = kpVelocityLimit;
   }

   public void setEffortLowerLimit(double effortLowerLimit)
   {
      this.effortLowerLimit = effortLowerLimit;
   }

   public void setEffortUpperLimit(double effortUpperLimit)
   {
      this.effortUpperLimit = effortUpperLimit;
   }

   public void setEffortLimits(double effortLowerLimit, double effortUpperLimit)
   {
      this.effortLowerLimit = effortLowerLimit;
      this.effortUpperLimit = effortUpperLimit;
   }

   public void setEffortLimits(double effortLimit)
   {
      this.effortLowerLimit = -effortLimit;
      this.effortUpperLimit = +effortLimit;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public void setStiction(double stiction)
   {
      this.stiction = stiction;
   }

   public double getPositionLowerLimit()
   {
      return positionLowerLimit;
   }

   public double getPositionUpperLimit()
   {
      return positionUpperLimit;
   }

   public double getKpPositionLimit()
   {
      return kpPositionLimit;
   }

   public double getKdPositionLimit()
   {
      return kdPositionLimit;
   }

   public double getVelocityLowerLimit()
   {
      return velocityLowerLimit;
   }

   public double getVelocityUpperLimit()
   {
      return velocityUpperLimit;
   }

   public double getKpVelocityLimit()
   {
      return kpVelocityLimit;
   }

   public double getEffortLowerLimit()
   {
      return effortLowerLimit;
   }

   public double getEffortUpperLimit()
   {
      return effortUpperLimit;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getStiction()
   {
      return stiction;
   }

   public Vector3D getAxis()
   {
      return axis;
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      double massScale = Math.pow(factor, massScalePower);
      damping = massScale * damping;

      kpPositionLimit = massScale * kpPositionLimit;
      kdPositionLimit = massScale * kdPositionLimit;

      kpVelocityLimit = massScale * kpVelocityLimit;

      super.scale(factor, massScalePower, ignoreInertiaScaleJointList);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      super.applyTransform(transform);
      transform.transform(axis);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      super.applyInverseTransform(transform);
      transform.inverseTransform(axis);
   }

   @Override
   public OneDoFJointDescription copy()
   {
      return new OneDoFJointDescription(this);
   }
}
