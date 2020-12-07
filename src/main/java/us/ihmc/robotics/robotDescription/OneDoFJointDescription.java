package us.ihmc.robotics.robotDescription;

import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class OneDoFJointDescription extends JointDescription
{
   private boolean containsLimitStops;
   private double qMin = Double.NEGATIVE_INFINITY;
   private double qMax = Double.POSITIVE_INFINITY;
   private double kLimit, bLimit;

   private double effortLimit = Double.POSITIVE_INFINITY;

   private double velocityLimit = Double.POSITIVE_INFINITY;
   private double velocityDamping;

   private double damping;
   private double stiction;

   private final Vector3D jointAxis = new Vector3D();

   public OneDoFJointDescription(String name, Tuple3DReadOnly offset, Vector3DReadOnly jointAxis)
   {
      super(name, offset);
      this.jointAxis.set(jointAxis);
   }

   public OneDoFJointDescription(OneDoFJointDescription other)
   {
      super(other);
      jointAxis.set(other.jointAxis);

      containsLimitStops = other.containsLimitStops;
      qMin = other.qMin;
      qMax = other.qMax;
      kLimit = other.kLimit;
      bLimit = other.bLimit;
      effortLimit = other.effortLimit;
      velocityLimit = other.velocityLimit;
      velocityDamping = other.velocityDamping;
      damping = other.damping;
      stiction = other.stiction;
   }

   public void setVelocityLimits(double velocityLimit, double velocityDamping)
   {
      this.velocityLimit = velocityLimit;
      this.velocityDamping = velocityDamping;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getStiction()
   {
      return stiction;
   }

   public double getVelocityLimit()
   {
      return velocityLimit;
   }

   public double getVelocityDamping()
   {
      return velocityDamping;
   }

   public void setStiction(double stiction)
   {
      this.stiction = stiction;
   }

   public void setLimitStops(double qMin, double qMax, double kLimit, double bLimit)
   {
      containsLimitStops = true;
      this.qMin = qMin;
      this.qMax = qMax;
      this.kLimit = kLimit;
      this.bLimit = bLimit;
   }

   public Vector3DReadOnly getJointAxis()
   {
      return jointAxis;
   }

   public void getJointAxis(Vector3DBasics jointAxisToPack)
   {
      jointAxisToPack.set(jointAxis);
   }

   public boolean containsLimitStops()
   {
      return containsLimitStops;
   }

   public double[] getLimitStopParameters()
   {
      return new double[] {qMin, qMax, kLimit, bLimit};
   }

   public double getLowerLimit()
   {
      return qMin;
   }

   public double getUpperLimit()
   {
      return qMax;
   }

   public void setEffortLimit(double effortLimit)
   {
      this.effortLimit = effortLimit;
   }

   public double getEffortLimit()
   {
      return effortLimit;
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      double massScale = Math.pow(factor, massScalePower);
      damping = massScale * damping;

      kLimit = massScale * kLimit;
      bLimit = massScale * bLimit;

      velocityDamping = massScale * velocityDamping;

      super.scale(factor, massScalePower, ignoreInertiaScaleJointList);
   }

   @Override
   public OneDoFJointDescription copy()
   {
      return new OneDoFJointDescription(this);
   }
}
