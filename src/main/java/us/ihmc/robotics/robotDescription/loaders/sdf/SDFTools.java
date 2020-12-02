package us.ihmc.robotics.robotDescription.loaders.sdf;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.VisualDescription;
import us.ihmc.graphicsDescription.appearance.MaterialDescription;
import us.ihmc.graphicsDescription.color.ColorDescription;
import us.ihmc.graphicsDescription.geometry.Box3DDescription;
import us.ihmc.graphicsDescription.geometry.Cylinder3DDescription;
import us.ihmc.graphicsDescription.geometry.GeometryDescription;
import us.ihmc.graphicsDescription.geometry.ModelFileGeometryDescription;
import us.ihmc.graphicsDescription.geometry.Sphere3DDescription;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.robotics.robotDescription.joints.FixedJointDescription;
import us.ihmc.robotics.robotDescription.joints.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.joints.JointDescription;
import us.ihmc.robotics.robotDescription.joints.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.joints.PinJointDescription;
import us.ihmc.robotics.robotDescription.joints.SliderJointDescription;
import us.ihmc.robotics.robotDescription.links.LinkDescription;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFGeometry;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFInertia;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis.SDFLimit;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFLink;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual.SDFMaterial;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class SDFTools
{
   private static final double DEFAULT_MASS = 0.0;
   private static final double DEFAULT_IXX = 0.0;
   private static final double DEFAULT_IYY = 0.0;
   private static final double DEFAULT_IZZ = 0.0;
   private static final double DEFAULT_IXY = 0.0;
   private static final double DEFAULT_IXZ = 0.0;
   private static final double DEFAULT_IYZ = 0.0;

   private static final UnitVector3DReadOnly DEFAULT_AXIS = new UnitVector3D(Axis3D.X);
   private static final double DEFAULT_LOWER_LIMIT = Double.NEGATIVE_INFINITY;
   private static final double DEFAULT_UPPER_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_EFFORT_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_VELOCITY_LIMIT = Double.POSITIVE_INFINITY;

   public static LinkDescription connectKinematics(List<LinkDescription> rigidBodyDefinitions, List<JointDescription> jointDefinitions,
                                                   List<SDFJoint> sdfJoints, List<SDFLink> sdfLinks)
   {
      Map<String, SDFLink> sdfLinkMap = sdfLinks.stream().collect(Collectors.toMap(SDFLink::getName, Function.identity()));
      Map<String, LinkDescription> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                .collect(Collectors.toMap(LinkDescription::getName, Function.identity()));
      Map<String, JointDescription> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));

      if (sdfJoints != null)
      {
         for (SDFJoint sdfJoint : sdfJoints)
         {
            String parent = sdfJoint.getParent();
            String child = sdfJoint.getChild();
            LinkDescription parentLinkDescription = rigidBodyDefinitionMap.get(parent);
            LinkDescription childLinkDescription = rigidBodyDefinitionMap.get(child);
            JointDescription jointDefinition = jointDefinitionMap.get(sdfJoint.getName());
            jointDefinition.getTransformToParentJoint().set(parsePose(sdfLinkMap.get(sdfJoint.getParent()).getPose()));

            jointDefinition.setLink(childLinkDescription);
            jointDefinition.getChildrenJoints().add(jointDefinition);
         }
      }

      if (sdfJoints == null)
      {
         return rigidBodyDefinitions.get(0);
      }
      else
      {
         Map<String, SDFJoint> childToParentJoint = sdfJoints.stream().collect(Collectors.toMap(SDFJoint::getChild, Function.identity()));

         String rootLinkName = sdfJoints.get(0).getParent();
         SDFJoint parentJoint = childToParentJoint.get(rootLinkName);

         while (parentJoint != null)
         {
            rootLinkName = parentJoint.getParent();
            parentJoint = childToParentJoint.get(rootLinkName);
         }

         for (SDFJoint sdfJoint : sdfJoints)
         {
            String jointName = sdfJoint.getName();
            JointDescription jointDefinition = jointDefinitionMap.get(jointName);

            String parentLinkName = sdfJoint.getParent();
            String childLinkName = sdfJoint.getChild();
            SDFJoint parentSDFJoint = childToParentJoint.get(parentLinkName);
            SDFLink parentSDFLink = sdfLinkMap.get(parentLinkName);
            SDFLink childSDFLink = sdfLinkMap.get(childLinkName);

            RigidBodyTransform parentLinkPose = parsePose(parentSDFLink.getPose());
            RigidBodyTransform childLinkPose = parsePose(childSDFLink.getPose());
            RigidBodyTransform parentJointParsedPose = parsePose(parentSDFJoint != null ? parentSDFJoint.getPose() : null);
            RigidBodyTransform jointParsedPose = parsePose(sdfJoint.getPose());

            RigidBodyTransform parentJointPose = new RigidBodyTransform(parentLinkPose);
            parentJointPose.multiply(parentJointParsedPose);

            RigidBodyTransform jointPose = new RigidBodyTransform(childLinkPose);
            jointPose.multiply(jointParsedPose);

            RigidBodyTransform transformToParentJoint = jointDefinition.getTransformToParentJoint();
            transformToParentJoint.setAndInvert(parentJointPose);
            transformToParentJoint.multiply(jointPose);

            jointDefinition.getTransformToParentJoint().getRotation().setToZero();
            parentLinkPose.transform(jointDefinition.getTransformToParentJoint().getTranslation());

            LinkDescription childDefinition = rigidBodyDefinitionMap.get(childSDFLink.getName());
            RigidBodyTransform inertiaPose = childDefinition.getInertiaPose();
            Vector3DBasics comOffset = childDefinition.getInertiaPose().getTranslation();
            childLinkPose.transform(comOffset);
            inertiaPose.transform(childDefinition.getMomentOfInertia());
            childLinkPose.transform(childDefinition.getMomentOfInertia());
            inertiaPose.getRotation().setToZero();
            for (VisualDescription visualDefinition : childDefinition.getVisualDefinitions())
            {
               RigidBodyTransform visualPose = visualDefinition.getPose();
               childLinkPose.getRotation().transform(visualPose.getRotation());
               childLinkPose.getRotation().transform(visualPose.getTranslation());
            }
         }

         return rigidBodyDefinitionMap.get(rootLinkName);
      }
   }

   private static PinJointDescription toRevoluteJointDefinition(SDFJoint sdfJoint, boolean ignorePositionLimits)
   {
      PinJointDescription definition = new PinJointDescription(sdfJoint.getName());

      definition.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      definition.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), definition, ignorePositionLimits);

      return definition;
   }

   private static SliderJointDescription toPrismaticJointDefinition(SDFJoint sdfJoint)
   {
      SliderJointDescription definition = new SliderJointDescription(sdfJoint.getName());
      definition.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      definition.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), definition, false);

      return definition;
   }

   private static FixedJointDescription toFixedJoint(SDFJoint sdfJoint)
   {
      FixedJointDescription definition = new FixedJointDescription(sdfJoint.getName());

      RigidBodyTransform parseRigidBodyTransform = parsePose(sdfJoint.getPose());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform);

      return definition;
   }

   private static FloatingPlanarJointDescription toPlanarJointDefinition(SDFJoint sdfJoint)
   {
      Vector3D surfaceNormal = parseAxis(sdfJoint.getAxis());

      Plane plane;

      if (surfaceNormal.geometricallyEquals(Axis3D.X, 1.0e-5))
         plane = Plane.YZ;
      else if (surfaceNormal.geometricallyEquals(Axis3D.Y, 1.0e-5))
         plane = Plane.XZ;
      else if (surfaceNormal.geometricallyEquals(Axis3D.Z, 1.0e-5))
         plane = Plane.XY;
      else
         throw new UnsupportedOperationException("Planar joint are supported only with a surface normal equal to: "
               + EuclidCoreIOTools.getTuple3DString(Axis3D.Y) + ", received:" + surfaceNormal);

      FloatingPlanarJointDescription definition = new FloatingPlanarJointDescription(sdfJoint.getName(), plane);
      definition.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));

      return definition;
   }

   public static VisualDescription toVisualDefinition(SDFVisual sdfVisual)
   {
      if (sdfVisual == null)
         return null;

      VisualDescription visualDefinition = new VisualDescription();
      visualDefinition.setName(sdfVisual.getName());
      visualDefinition.setPose(parsePose(sdfVisual.getPose()));
      visualDefinition.setMaterial(toMaterialDefinition(sdfVisual.getMaterial()));
      visualDefinition.setGeometry(toGeometryDefinition(sdfVisual.getGeometry()));
      return visualDefinition;
   }

   public static GeometryDescription toGeometryDefinition(SDFGeometry sdfGeometry)
   {
      return toGeometryDefinition(sdfGeometry, Collections.emptyList());
   }

   public static GeometryDescription toGeometryDefinition(SDFGeometry sdfGeometry, List<String> resourceDirectories)
   {
      if (sdfGeometry.getBox() != null)
      {
         Box3DDescription boxGeometryDefinition = new Box3DDescription();
         boxGeometryDefinition.setSize(parseVector3D(sdfGeometry.getBox().getSize(), null));
         return boxGeometryDefinition;
      }
      if (sdfGeometry.getCylinder() != null)
      {
         Cylinder3DDescription cylinderGeometryDefinition = new Cylinder3DDescription();
         cylinderGeometryDefinition.setRadius(parseDouble(sdfGeometry.getCylinder().getRadius(), 0.0));
         cylinderGeometryDefinition.setHeight(parseDouble(sdfGeometry.getCylinder().getLength(), 0.0));
         return cylinderGeometryDefinition;
      }
      if (sdfGeometry.getSphere() != null)
      {
         Sphere3DDescription sphereGeometryDefinition = new Sphere3DDescription();
         sphereGeometryDefinition.setRadius(parseDouble(sdfGeometry.getSphere().getRadius(), 0.0));
         return sphereGeometryDefinition;
      }
      if (sdfGeometry.getMesh() != null)
      {
         ModelFileGeometryDescription modelFileGeometryDefinition = new ModelFileGeometryDescription();
         modelFileGeometryDefinition.getResourceDirectories().addAll(resourceDirectories);
         modelFileGeometryDefinition.setFileName(sdfGeometry.getMesh().getUri());
         modelFileGeometryDefinition.setScale(parseVector3D(sdfGeometry.getMesh().getScale(), new Vector3D(1, 1, 1)));
         return modelFileGeometryDefinition;
      }
      throw new IllegalArgumentException("The given SDF Geometry is empty.");
   }

   public static MaterialDescription toMaterialDefinition(SDFMaterial sdfMaterial)
   {
      if (sdfMaterial == null)
         return null;

      MaterialDescription materialDefinition = new MaterialDescription();
      materialDefinition.setAmbientColor(toColorDefinition(sdfMaterial.getAmbient()));
      materialDefinition.setDiffuseColor(toColorDefinition(sdfMaterial.getDiffuse()));
      materialDefinition.setSpecularColor(toColorDefinition(sdfMaterial.getSpecular()));
      materialDefinition.setEmissiveColor(toColorDefinition(sdfMaterial.getEmissive()));
      // TODO handle the script
      return materialDefinition;
   }

   public static ColorDescription toColorDefinition(String sdfColor)
   {
      if (sdfColor == null)
         return null;
      double[] components = parseArray(sdfColor, null);
      if (components.length == 3)
         return ColorDescription.rgb(components);
      else
         return ColorDescription.rgba(components);
   }

   public static RigidBodyTransform parsePose(String pose)
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();

      if (pose != null)
      {
         String[] split = pose.split("\\s+");
         Vector3D position = new Vector3D(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]));
         YawPitchRoll orientation = new YawPitchRoll(Double.parseDouble(split[5]), Double.parseDouble(split[4]), Double.parseDouble(split[3]));
         rigidBodyTransform.set(orientation, position);
      }
      return rigidBodyTransform;
   }

   public static Matrix3D parseMomentOfInertia(SDFInertia inertia)
   {
      if (inertia == null)
         inertia = new SDFInertia();

      Matrix3D momentOfInertia = new Matrix3D();

      double ixx = parseDouble(inertia.getIxx(), DEFAULT_IXX);
      double iyy = parseDouble(inertia.getIyy(), DEFAULT_IYY);
      double izz = parseDouble(inertia.getIzz(), DEFAULT_IZZ);

      double ixy = parseDouble(inertia.getIxy(), DEFAULT_IXY);
      double ixz = parseDouble(inertia.getIxz(), DEFAULT_IXZ);
      double iyz = parseDouble(inertia.getIyz(), DEFAULT_IYZ);

      momentOfInertia.setM00(ixx);
      momentOfInertia.setM11(iyy);
      momentOfInertia.setM22(izz);

      momentOfInertia.setM01(ixy);
      momentOfInertia.setM02(ixz);
      momentOfInertia.setM12(iyz);

      momentOfInertia.setM10(ixy);
      momentOfInertia.setM20(ixz);
      momentOfInertia.setM21(iyz);

      return momentOfInertia;
   }

   public static void parseLimit(SDFLimit sdfLimit, OneDoFJointDescription jointDescriptionToParseLimitInto, boolean ignorePositionLimits)
   {
      double lowerLimit, upperLimit, effortLimit, velocityLimit;

      if (sdfLimit != null)
      {
         if (ignorePositionLimits)
         {
            lowerLimit = DEFAULT_LOWER_LIMIT;
            upperLimit = DEFAULT_UPPER_LIMIT;
         }
         else
         {
            lowerLimit = parseDouble(sdfLimit.getLower(), DEFAULT_LOWER_LIMIT);
            upperLimit = parseDouble(sdfLimit.getUpper(), DEFAULT_UPPER_LIMIT);
         }
         effortLimit = parseDouble(sdfLimit.getEffort(), DEFAULT_EFFORT_LIMIT);
         velocityLimit = parseDouble(sdfLimit.getVelocity(), DEFAULT_VELOCITY_LIMIT);
      }
      else
      {
         lowerLimit = DEFAULT_LOWER_LIMIT;
         upperLimit = DEFAULT_UPPER_LIMIT;
         effortLimit = DEFAULT_EFFORT_LIMIT;
         velocityLimit = DEFAULT_VELOCITY_LIMIT;
      }

      jointDescriptionToParseLimitInto.setPositionLimits(lowerLimit, upperLimit);
      jointDescriptionToParseLimitInto.setEffortLimits(effortLimit);
      jointDescriptionToParseLimitInto.setVelocityLimits(velocityLimit);
   }

   public static Vector3D parseAxis(SDFAxis axis)
   {
      if (axis == null)
         return new Vector3D(DEFAULT_AXIS);
      Vector3D parsedAxis = parseVector3D(axis.getXYZ(), new Vector3D(DEFAULT_AXIS));
      parsedAxis.normalize();
      return parsedAxis;
   }

   public static double parseDouble(String value, double defaultValue)
   {
      if (value == null)
         return defaultValue;
      return Double.parseDouble(value);
   }

   public static Vector3D parseVector3D(String value, Vector3D defaultValue)
   {
      if (value == null)
         return defaultValue;

      String[] split = value.split("\\s+");
      Vector3D vector = new Vector3D();
      vector.setX(Double.parseDouble(split[0]));
      vector.setY(Double.parseDouble(split[1]));
      vector.setZ(Double.parseDouble(split[2]));
      return vector;
   }

   public static double[] parseArray(String value, double[] defaultValue)
   {
      if (value == null)
         return defaultValue;

      String[] split = value.split("\\s+");
      double[] array = new double[split.length];

      for (int i = 0; i < split.length; i++)
         array[i] = Double.parseDouble(split[i]);

      return array;
   }
}
