package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class IMUSensorDescription extends SensorDescription
{
   private double accelerationNoiseMean;
   private double accelerationNoiseStandardDeviation;
   private double accelerationBiasMean;
   private double accelerationBiasStandardDeviation;

   private double angularVelocityNoiseMean;
   private double angularVelocityNoiseStandardDeviation;
   private double angularVelocityBiasMean;
   private double angularVelocityBiasStandardDeviation;

   public IMUSensorDescription(String name, RigidBodyTransform imuTransform)
   {
      super(name, imuTransform);
   }

   public IMUSensorDescription(IMUSensorDescription other)
   {
      super(other);

      accelerationNoiseMean = other.accelerationNoiseMean;
      accelerationNoiseStandardDeviation = other.accelerationNoiseStandardDeviation;
      accelerationBiasMean = other.accelerationBiasMean;
      accelerationBiasStandardDeviation = other.accelerationBiasStandardDeviation;

      angularVelocityNoiseMean = other.angularVelocityNoiseMean;
      angularVelocityNoiseStandardDeviation = other.angularVelocityNoiseStandardDeviation;
      angularVelocityBiasMean = other.angularVelocityBiasMean;
      angularVelocityBiasStandardDeviation = other.angularVelocityBiasStandardDeviation;
   }

   public void setAccelerationNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      setAccelerationNoiseMean(noiseMean);
      setAccelerationNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAccelerationBiasParameters(double biasMean, double biasStandardDeviation)
   {
      setAccelerationBiasMean(biasMean);
      setAccelerationBiasStandardDeviation(biasStandardDeviation);
   }

   public void setAngularVelocityNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      setAngularVelocityNoiseMean(noiseMean);
      setAngularVelocityNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAngularVelocityBiasParameters(double biasMean, double biasStandardDeviation)
   {
      setAngularVelocityNoiseMean(biasMean);
      setAngularVelocityNoiseStandardDeviation(biasStandardDeviation);
   }

   public double getAccelerationNoiseMean()
   {
      return accelerationNoiseMean;
   }

   public void setAccelerationNoiseMean(double accelerationNoiseMean)
   {
      this.accelerationNoiseMean = accelerationNoiseMean;
   }

   public double getAccelerationNoiseStandardDeviation()
   {
      return accelerationNoiseStandardDeviation;
   }

   public void setAccelerationNoiseStandardDeviation(double accelerationNoiseStandardDeviation)
   {
      this.accelerationNoiseStandardDeviation = accelerationNoiseStandardDeviation;
   }

   public double getAccelerationBiasMean()
   {
      return accelerationBiasMean;
   }

   public void setAccelerationBiasMean(double accelerationBiasMean)
   {
      this.accelerationBiasMean = accelerationBiasMean;
   }

   public double getAccelerationBiasStandardDeviation()
   {
      return accelerationBiasStandardDeviation;
   }

   public void setAccelerationBiasStandardDeviation(double accelerationBiasStandardDeviation)
   {
      this.accelerationBiasStandardDeviation = accelerationBiasStandardDeviation;
   }

   public double getAngularVelocityNoiseMean()
   {
      return angularVelocityNoiseMean;
   }

   public void setAngularVelocityNoiseMean(double angularVelocityNoiseMean)
   {
      this.angularVelocityNoiseMean = angularVelocityNoiseMean;
   }

   public double getAngularVelocityNoiseStandardDeviation()
   {
      return angularVelocityNoiseStandardDeviation;
   }

   public void setAngularVelocityNoiseStandardDeviation(double angularVelocityNoiseStandardDeviation)
   {
      this.angularVelocityNoiseStandardDeviation = angularVelocityNoiseStandardDeviation;
   }

   public double getAngularVelocityBiasMean()
   {
      return angularVelocityBiasMean;
   }

   public void setAngularVelocityBiasMean(double angularVelocityBiasMean)
   {
      this.angularVelocityBiasMean = angularVelocityBiasMean;
   }

   public double getAngularVelocityBiasStandardDeviation()
   {
      return angularVelocityBiasStandardDeviation;
   }

   public void setAngularVelocityBiasStandardDeviation(double angularVelocityBiasStandardDeviation)
   {
      this.angularVelocityBiasStandardDeviation = angularVelocityBiasStandardDeviation;
   }

   @Override
   public IMUSensorDescription copy()
   {
      return new IMUSensorDescription(this);
   }
}
