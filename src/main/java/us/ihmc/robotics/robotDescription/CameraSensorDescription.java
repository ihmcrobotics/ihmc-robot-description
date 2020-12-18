package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class CameraSensorDescription extends SensorDescription
{
   private double fieldOfView;
   private double clipNear;
   private double clipFar;

   private int imageWidth;
   private int imageHeight;

   public CameraSensorDescription(String name, Tuple3DReadOnly offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public CameraSensorDescription(String name, RigidBodyTransformReadOnly transformToJoint)
   {
      super(name, transformToJoint);
   }

   public CameraSensorDescription(String name, RigidBodyTransformReadOnly transformToJoint, double fieldOfView, double clipNear, double clipFar)
   {
      super(name, transformToJoint);

      setFieldOfView(fieldOfView);

      setClipNear(clipNear);
      setClipFar(clipFar);
   }

   public CameraSensorDescription(CameraSensorDescription other)
   {
      super(other);

      fieldOfView = other.fieldOfView;
      clipNear = other.clipNear;
      clipFar = other.clipFar;
      imageWidth = other.imageWidth;
      imageHeight = other.imageHeight;
   }

   public double getFieldOfView()
   {
      return fieldOfView;
   }

   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   public double getClipNear()
   {
      return clipNear;
   }

   public void setClipNear(double clipNear)
   {
      this.clipNear = clipNear;
   }

   public double getClipFar()
   {
      return clipFar;
   }

   public void setClipFar(double clipFar)
   {
      this.clipFar = clipFar;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   @Override
   public CameraSensorDescription copy()
   {
      return new CameraSensorDescription(this);
   }
}
