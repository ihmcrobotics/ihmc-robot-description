package us.ihmc.robotics.robotDescription.loaders.sdf;

import java.io.File;
import java.io.InputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.AffineTransform;
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
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.joints.FixedJointDescription;
import us.ihmc.robotics.robotDescription.joints.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.joints.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.joints.JointDescription;
import us.ihmc.robotics.robotDescription.joints.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.joints.PinJointDescription;
import us.ihmc.robotics.robotDescription.joints.SliderJointDescription;
import us.ihmc.robotics.robotDescription.links.LinkDescription;
import us.ihmc.robotics.robotDescription.links.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFGeometry;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFInertia;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis.SDFLimit;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFLink;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFLink.SDFInertial;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFModel;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFRoot;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFURIHolder;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual.SDFMaterial;

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

   public static SDFRoot loadSDFRoot(File sdfFile) throws JAXBException
   {
      return loadSDFRoot(sdfFile, Collections.emptyList());
   }

   public static SDFRoot loadSDFRoot(File sdfFile, Collection<String> resourceDirectories) throws JAXBException
   {
      Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
      File parentFile = sdfFile.getParentFile();

      if (parentFile != null)
      {
         allResourceDirectories.add(parentFile.getAbsolutePath() + File.separator);
         Stream.of(parentFile.listFiles(File::isDirectory)).map(file -> file.getAbsolutePath() + File.separator).forEach(allResourceDirectories::add);
      }

      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(sdfFile);

      resolvePaths(sdfRoot, allResourceDirectories);

      return sdfRoot;
   }

   public static SDFRoot loadSDFRoot(InputStream inputStream, Collection<String> resourceDirectories) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(inputStream);

      resolvePaths(sdfRoot, resourceDirectories);

      return sdfRoot;
   }

   public static void resolvePaths(SDFRoot sdfRoot, Collection<String> resourceDirectories)
   {
      List<SDFURIHolder> uriHolders = sdfRoot.getURIHolders();

      for (SDFURIHolder sdfURIHolder : uriHolders)
      {
         sdfURIHolder.setUri(tryToConvertToPath(sdfURIHolder.getUri(), resourceDirectories));
      }
   }

   public static String tryToConvertToPath(String filename, Collection<String> resourceDirectories)
   {
      try
      {
         URI uri = new URI(filename);

         String authority = uri.getAuthority() == null ? "" : uri.getAuthority();

         for (String resourceDirectory : resourceDirectories)
         {
            String fullname = resourceDirectory + authority + uri.getPath();
            // Path relative to class root
            if (SDFTools.class.getClassLoader().getResource(fullname) != null)
            {
               return fullname;
            }
            // Absolute path
            if (new File(fullname).exists())
            {
               return fullname;
            }
         }

         // Let's look in the parent directories of the resources if we can find a match to authority
         String resourceContainingAuthority = null;

         for (String resourceDirectory : resourceDirectories)
         {
            if (resourceDirectory.contains(authority))
            {
               resourceContainingAuthority = resourceDirectory;
               break;
            }
         }

         if (resourceContainingAuthority != null)
         {
            int lastIndexOf = resourceContainingAuthority.lastIndexOf(authority, resourceContainingAuthority.length());
            String newResource = resourceContainingAuthority.substring(0, lastIndexOf);

            if (!resourceDirectories.contains(newResource))
            {
               resourceDirectories.add(newResource);
               return tryToConvertToPath(filename, resourceDirectories);
            }
         }
      }
      catch (URISyntaxException e)
      {
         System.err.println("Malformed resource path in SDF file for path: " + filename);
      }

      return null;
   }

   public static RobotDescription toFloatingRootJointRobotDescription(RobotDescription robotDefinition, SDFModel sdfModel)
   {
      return toRobotDefinition(new FloatingJointDescription(), sdfModel);
   }

   public static RobotDescription toRobotDefinition(JointDescription rootJoint, SDFModel sdfModel)
   {
      List<SDFLink> sdfLinks = sdfModel.getLinks();
      List<SDFJoint> sdfJoints = sdfModel.getJoints();

      List<LinkDescription> rigidBodyDefinitions = sdfLinks.stream().map(SDFTools::toRigidBodyDefinition).collect(Collectors.toList());
      List<JointDescription> jointDefinitions;
      if (sdfJoints == null)
         jointDefinitions = Collections.emptyList();
      else
         jointDefinitions = sdfJoints.stream().map(SDFTools::toJointDefinition).collect(Collectors.toList());
      LinkDescription rootBodyDefinition = connectKinematics(rigidBodyDefinitions, jointDefinitions, sdfJoints, sdfLinks);

      if (rootJoint.getName() == null)
         rootJoint.setName(rootBodyDefinition.getName());
      rootJoint.setLink(rootBodyDefinition);

      RobotDescription robotDefinition = new RobotDescription(sdfModel.getName());
      robotDefinition.addRootJoint(rootJoint);

      return robotDefinition;
   }

   public static LinkDescription toRigidBodyDefinition(SDFLink sdfLink)
   {
      LinkDescription description = new LinkDescription(sdfLink.getName());

      SDFInertial sdfInertial = sdfLink.getInertial();

      if (sdfInertial == null)
      {
         description.setMass(parseDouble(null, DEFAULT_MASS));
         description.getMomentOfInertia().set(parseMomentOfInertia(null));
         description.getInertiaPose().set(parsePose(null));
      }
      else
      {
         description.setMass(parseDouble(sdfInertial.getMass(), DEFAULT_MASS));
         description.getMomentOfInertia().set(parseMomentOfInertia(sdfInertial.getInertia()));
         description.getInertiaPose().set(parsePose(sdfInertial.getPose()));
      }

      if (sdfLink.getVisuals() != null)
      {
         LinkGraphicsDescription linkGraphicsDescription = new LinkGraphicsDescription();
         sdfLink.getVisuals().stream().map(SDFTools::toVisualDefinition).forEach(linkGraphicsDescription::addVisualDescription);
         description.setLinkGraphics(linkGraphicsDescription);
      }

      return description;
   }

   public static JointDescription toJointDefinition(SDFJoint sdfJoint)
   {
      switch (sdfJoint.getType())
      {
         case "continuous":
            return toRevoluteJointDefinition(sdfJoint, true);
         case "revolute":
            return toRevoluteJointDefinition(sdfJoint, false);
         case "prismatic":
            return toPrismaticJointDefinition(sdfJoint);
         case "fixed":
            return toFixedJoint(sdfJoint);
         case "floating":
            return toSixDoFJointDefinition(sdfJoint);
         case "planar":
            return toPlanarJointDefinition(sdfJoint);
         default:
            throw new RuntimeException("Unexpected value for the joint type: " + sdfJoint.getType());
      }
   }

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

            for (VisualDescription visualDescription : childDefinition.getLinkGraphics().getVisualDescriptions())
            {
               AffineTransform visualPose = visualDescription.getPose();
               visualPose.prependOrientation(childLinkPose.getRotation());
            }
         }

         return rigidBodyDefinitionMap.get(rootLinkName);
      }
   }

   private static PinJointDescription toRevoluteJointDefinition(SDFJoint sdfJoint, boolean ignorePositionLimits)
   {
      PinJointDescription description = new PinJointDescription(sdfJoint.getName());

      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      description.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), description, ignorePositionLimits);

      return description;
   }

   private static SliderJointDescription toPrismaticJointDefinition(SDFJoint sdfJoint)
   {
      SliderJointDescription description = new SliderJointDescription(sdfJoint.getName());
      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      description.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), description, false);

      return description;
   }

   private static FixedJointDescription toFixedJoint(SDFJoint sdfJoint)
   {
      FixedJointDescription description = new FixedJointDescription(sdfJoint.getName());
      RigidBodyTransform parseRigidBodyTransform = parsePose(sdfJoint.getPose());
      description.getTransformToParentJoint().set(parseRigidBodyTransform);
      return description;
   }

   private static FloatingJointDescription toSixDoFJointDefinition(SDFJoint sdfJoint)
   {
      FloatingJointDescription description = new FloatingJointDescription(sdfJoint.getName());
      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      return description;
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

      FloatingPlanarJointDescription description = new FloatingPlanarJointDescription(sdfJoint.getName(), plane);
      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));

      return description;
   }

   public static VisualDescription toVisualDefinition(SDFVisual sdfVisual)
   {
      if (sdfVisual == null)
         return null;

      VisualDescription visualDescription = new VisualDescription();
      visualDescription.setName(sdfVisual.getName());
      visualDescription.setPose(new AffineTransform(parsePose(sdfVisual.getPose())));
      visualDescription.setMaterial(toMaterialDefinition(sdfVisual.getMaterial()));
      visualDescription.setGeometry(toGeometryDefinition(sdfVisual.getGeometry()));
      return visualDescription;
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
