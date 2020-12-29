package us.ihmc.robotics.robotDescription.joints;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.robotDescription.RobotDescriptionNode;
import us.ihmc.robotics.robotDescription.links.LinkDescription;
import us.ihmc.robotics.robotDescription.sensors.SensorDescription;
import us.ihmc.robotics.robotDescription.simulation.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.simulation.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.simulation.KinematicPointDescription;

public class JointDescription implements RobotDescriptionNode
{
   private String name;
   private JointDescription parentJoint;
   private final RigidBodyTransform transformToParentJoint = new RigidBodyTransform();

   private final List<JointDescription> childrenJointDescriptions = new ArrayList<>();
   private final List<LoopClosureConstraintDescription> childrenConstraintDescriptions = new ArrayList<>();

   private LinkDescription link;

   // Lists of kinematic points on the robot. When adding types of kinematic points, make sure to update the getAllKinematicPoints(List<KinematicPointDescription>) function
   private final List<KinematicPointDescription> kinematicPoints = new ArrayList<>();
   private final List<ExternalForcePointDescription> externalForcePoints = new ArrayList<>();
   private final List<GroundContactPointDescription> groundContactPoints = new ArrayList<>();

   private final List<SensorDescription> sensors = new ArrayList<>();

   private boolean isDynamic = true;

   public JointDescription()
   {
   }

   public JointDescription(String name)
   {
      this.name = name;
   }

   public JointDescription(String name, Tuple3DReadOnly offsetFromParentJoint)
   {
      this.name = name;
      transformToParentJoint.getTranslation().set(offsetFromParentJoint);
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public JointDescription(JointDescription other)
   {
      this.name = other.name;
      transformToParentJoint.set(other.transformToParentJoint);
      link = other.link == null ? null : other.link.copy();

      other.childrenConstraintDescriptions.forEach(kp -> childrenConstraintDescriptions.add(kp.copy()));
      childrenConstraintDescriptions.forEach(e -> e.setParentJoint(this));
      other.kinematicPoints.forEach(kp -> kinematicPoints.add(kp.copy()));
      other.externalForcePoints.forEach(efp -> externalForcePoints.add(efp.copy()));
      other.groundContactPoints.forEach(gcp -> groundContactPoints.add(gcp.copy()));

      other.sensors.forEach(sensor -> sensors.add(sensor.copy()));

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

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public RigidBodyTransform getTransformToParentJoint()
   {
      return transformToParentJoint;
   }

   public LinkDescription getLink()
   {
      return link;
   }

   public void setLink(LinkDescription link)
   {
      if (this.link != null)
         this.link.setParentJoint(null);

      this.link = link;

      if (link != null)
         link.setParentJoint(this);
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

   public void addSensor(SensorDescription sensorDescription)
   {
      sensors.add(sensorDescription);
   }

   public List<SensorDescription> getSensors()
   {
      return sensors;
   }

   public <T extends SensorDescription> List<T> getSensors(Class<T> sensorType)
   {
      return sensors.stream().filter(sensorType::isInstance).map(sensorType::cast).collect(Collectors.toList());
   }

   public void setIsDynamic(boolean isDynamic)
   {
      this.isDynamic = isDynamic;
   }

   public boolean isDynamic()
   {
      return isDynamic;
   }

   public void getAllKinematicPoints(List<KinematicPointDescription> allKinematicPoints)
   {
      allKinematicPoints.addAll(kinematicPoints);
      allKinematicPoints.addAll(externalForcePoints);
      allKinematicPoints.addAll(groundContactPoints);
   }

   public static void scaleChildrenJoint(List<JointDescription> childrenJoints, double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointDescription description = childrenJoints.get(i);
         description.getTransformToParentJoint().getTranslation().scale(factor);
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
      for (int i = 0; i < sensors.size(); i++)
      {
         SensorDescription sensor = sensors.get(i);
         RigidBodyTransform transformToJoint = sensor.getTransformToJoint();
         transformToJoint.getTranslation().scale(factor);
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
