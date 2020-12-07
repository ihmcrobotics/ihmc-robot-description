package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class JointDescription implements RobotDescriptionNode
{
   private final String name;
   private final List<JointDescription> childrenJointDescriptions = new ArrayList<>();
   private final List<LoopClosureConstraintDescription> childrenConstraintDescriptions = new ArrayList<>();

   private JointDescription parentJoint;
   private final Vector3D offsetFromParentJoint = new Vector3D();

   private LinkDescription link;

   // Lists of kinematic points on the robot. When adding types of kinematic points, make sure to update the getAllKinematicPoints(List<KinematicPointDescription>) function
   private final List<KinematicPointDescription> kinematicPoints = new ArrayList<>();
   private final List<ExternalForcePointDescription> externalForcePoints = new ArrayList<>();
   private final List<GroundContactPointDescription> groundContactPoints = new ArrayList<>();

   // Lists of sensors. When adding sensors, make sure to update the getSensors(List<SensorDescription>) function.
   private final List<JointWrenchSensorDescription> wrenchSensors = new ArrayList<>();
   private final List<CameraSensorDescription> cameraSensors = new ArrayList<>();
   private final List<IMUSensorDescription> imuSensors = new ArrayList<>();
   private final List<LidarSensorDescription> lidarSensors = new ArrayList<>();
   private final List<ForceSensorDescription> forceSensors = new ArrayList<>();

   private boolean isDynamic = true;

   public JointDescription(String name, Tuple3DReadOnly offsetFromParentJoint)
   {
      this.name = name;
      this.offsetFromParentJoint.set(offsetFromParentJoint);
   }

   public JointDescription(JointDescription other)
   {
      this.name = other.name;
      offsetFromParentJoint.set(other.offsetFromParentJoint);
      link = other.link == null ? null : other.link.copy();

      other.childrenConstraintDescriptions.forEach(kp -> childrenConstraintDescriptions.add(kp.copy()));
      childrenConstraintDescriptions.forEach(e -> e.setParentJoint(this));
      other.kinematicPoints.forEach(kp -> kinematicPoints.add(kp.copy()));
      other.externalForcePoints.forEach(efp -> externalForcePoints.add(efp.copy()));
      other.groundContactPoints.forEach(gcp -> groundContactPoints.add(gcp.copy()));

      other.wrenchSensors.forEach(sensor -> wrenchSensors.add(sensor.copy()));
      other.cameraSensors.forEach(sensor -> cameraSensors.add(sensor.copy()));
      other.imuSensors.forEach(sensor -> imuSensors.add(sensor.copy()));
      other.lidarSensors.forEach(sensor -> lidarSensors.add(sensor.copy()));
      other.forceSensors.forEach(sensor -> forceSensors.add(sensor.copy()));

      isDynamic = other.isDynamic;
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setParentJoint(JointDescription parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public void setOffsetFromParentJoint(Tuple3DReadOnly offset)
   {
      offsetFromParentJoint.set(offset);
   }

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public Vector3DReadOnly getOffsetFromParentJoint()
   {
      return offsetFromParentJoint;
   }

   public void getOffsetFromParentJoint(Tuple3DBasics offsetToPack)
   {
      offsetToPack.set(offsetFromParentJoint);
   }

   public LinkDescription getLink()
   {
      return link;
   }

   public void setLink(LinkDescription link)
   {
      this.link = link;
   }

   public void addJoint(JointDescription childJointDescription)
   {
      childrenJointDescriptions.add(childJointDescription);

      if (childJointDescription.getParentJoint() != null)
      {
         throw new RuntimeException("JointDescription " + childJointDescription.getName() + "already has a parent joint: "
               + childJointDescription.getParentJoint().getName());
      }

      childJointDescription.setParentJoint(this);
   }

   public boolean removeJoint(JointDescription childJointDescription)
   {
      return childrenJointDescriptions.remove(childJointDescription);
   }

   public void addConstraint(LoopClosureConstraintDescription childConstraintDescription)
   {
      childrenConstraintDescriptions.add(childConstraintDescription);

      if (childConstraintDescription.getParentJoint() != null)
      {
         throw new RuntimeException("LoopClosureConstraintDescription " + childConstraintDescription.getName() + "already has a parent joint: "
               + childConstraintDescription.getParentJoint().getName());
      }

      childConstraintDescription.setParentJoint(this);
   }

   @Override
   public List<JointDescription> getChildrenJoints()
   {
      return childrenJointDescriptions;
   }

   public List<LoopClosureConstraintDescription> getChildrenConstraintDescriptions()
   {
      return childrenConstraintDescriptions;
   }

   public void addGroundContactPoint(GroundContactPointDescription groundContactPointDescription)
   {
      groundContactPoints.add(groundContactPointDescription);
   }

   public List<GroundContactPointDescription> getGroundContactPoints()
   {
      return groundContactPoints;
   }

   public void addExternalForcePoint(ExternalForcePointDescription externalForcePointDescription)
   {
      externalForcePoints.add(externalForcePointDescription);
   }

   public List<ExternalForcePointDescription> getExternalForcePoints()
   {
      return externalForcePoints;
   }

   public void addKinematicPoint(KinematicPointDescription kinematicPointDescription)
   {
      kinematicPoints.add(kinematicPointDescription);
   }

   public List<KinematicPointDescription> getKinematicPoints()
   {
      return kinematicPoints;
   }

   public void addJointWrenchSensor(JointWrenchSensorDescription jointWrenchSensorDescription)
   {
      wrenchSensors.add(jointWrenchSensorDescription);
   }

   public List<JointWrenchSensorDescription> getWrenchSensors()
   {
      return wrenchSensors;
   }

   public void addCameraSensor(CameraSensorDescription cameraSensorDescription)
   {
      cameraSensors.add(cameraSensorDescription);
   }

   public List<CameraSensorDescription> getCameraSensors()
   {
      return cameraSensors;
   }

   public void addIMUSensor(IMUSensorDescription imuSensorDescription)
   {
      imuSensors.add(imuSensorDescription);
   }

   public List<IMUSensorDescription> getIMUSensors()
   {
      return imuSensors;
   }

   public void addLidarSensor(LidarSensorDescription lidarSensor)
   {
      lidarSensors.add(lidarSensor);
   }

   public List<LidarSensorDescription> getLidarSensors()
   {
      return lidarSensors;
   }

   public void addForceSensor(ForceSensorDescription forceSensor)
   {
      forceSensors.add(forceSensor);
   }

   public List<ForceSensorDescription> getForceSensors()
   {
      return forceSensors;
   }

   public void setIsDynamic(boolean isDynamic)
   {
      this.isDynamic = isDynamic;
   }

   public boolean isDynamic()
   {
      return isDynamic;
   }

   public void getSensors(List<SensorDescription> sensors)
   {
      sensors.addAll(wrenchSensors);
      sensors.addAll(cameraSensors);
      sensors.addAll(imuSensors);
      sensors.addAll(lidarSensors);
      sensors.addAll(forceSensors);
   }

   public void getAllKinematicPoints(List<KinematicPointDescription> allKinematicPoints)
   {
      allKinematicPoints.addAll(kinematicPoints);
      allKinematicPoints.addAll(externalForcePoints);
      allKinematicPoints.addAll(groundContactPoints);
   }

   public static void scaleChildrenJoint(List<JointDescription> childrenJoints, double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      Vector3D offsetFromParentJoint = new Vector3D();
      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointDescription description = childrenJoints.get(i);

         description.getOffsetFromParentJoint(offsetFromParentJoint);
         offsetFromParentJoint.scale(factor);
         description.setOffsetFromParentJoint(offsetFromParentJoint);

         description.scale(factor, massScalePower, ignoreInertiaScaleJointList);
      }

   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      scaleSensorsOffsets(factor);
      scaleAllKinematicsPointOffsets(factor);

      boolean scaleInertia = true;
      if (ignoreInertiaScaleJointList.contains(getName()))
      {
         scaleInertia = false;
      }
      link.scale(factor, massScalePower, scaleInertia);
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower, ignoreInertiaScaleJointList);
   }

   private void scaleSensorsOffsets(double factor)
   {
      List<SensorDescription> sensors = new ArrayList<>();
      getSensors(sensors);

      for (int i = 0; i < sensors.size(); i++)
      {
         SensorDescription sensor = sensors.get(i);
         RigidBodyTransform transformToJoint = sensor.getTransformToJoint();
         Vector3D translation = new Vector3D();
         translation.set(transformToJoint.getTranslation());
         translation.scale(factor);
         transformToJoint.getTranslation().set(translation);
         sensor.setTransformToJoint(transformToJoint);
      }
   }

   private void scaleAllKinematicsPointOffsets(double factor)
   {
      List<KinematicPointDescription> allKinematicPoints = new ArrayList<>();
      getAllKinematicPoints(allKinematicPoints);
      for (int i = 0; i < allKinematicPoints.size(); i++)
      {
         KinematicPointDescription kinematicPoint = allKinematicPoints.get(i);

         Vector3D offset = kinematicPoint.getOffsetFromJoint();
         offset.scale(factor);
         kinematicPoint.setOffsetFromJoint(offset);
      }
   }

   @Override
   public JointDescription copy()
   {
      return new JointDescription(this);
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ", joint: " + name;
   }
}
