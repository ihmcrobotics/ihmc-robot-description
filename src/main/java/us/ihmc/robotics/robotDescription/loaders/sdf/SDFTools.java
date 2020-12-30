package us.ihmc.robotics.robotDescription.loaders.sdf;

import java.io.File;
import java.io.InputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
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
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.VisualDescription;
import us.ihmc.graphicsDescription.appearance.MaterialDescription;
import us.ihmc.graphicsDescription.color.ColorDescription;
import us.ihmc.graphicsDescription.geometry.Box3DDescription;
import us.ihmc.graphicsDescription.geometry.Cylinder3DDescription;
import us.ihmc.graphicsDescription.geometry.GeometryDescription;
import us.ihmc.graphicsDescription.geometry.ModelFileGeometryDescription;
import us.ihmc.graphicsDescription.geometry.Sphere3DDescription;
import us.ihmc.log.LogTools;
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
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis.SDFDynamics;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFJoint.SDFAxis.SDFLimit;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFLink;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFLink.SDFInertial;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFModel;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFRoot;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFCamera;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFIMU;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFIMU.SDFIMUNoise;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFIMU.SDFIMUNoise.SDFNoiseParameters;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFRay;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFRay.SDFNoise;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFRay.SDFRange;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFRay.SDFScan.SDFHorizontalScan;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFSensor.SDFRay.SDFScan.SDFVerticalScan;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFURIHolder;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual;
import us.ihmc.robotics.robotDescription.loaders.sdf.items.SDFVisual.SDFMaterial;
import us.ihmc.robotics.robotDescription.sensors.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.SensorDescription;

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

   public static RobotDescription toFloatingRootJointRobotDescription(SDFModel sdfModel)
   {
      return toRobotDefinition(new FloatingJointDescription(), sdfModel);
   }

   public static RobotDescription toRobotDefinition(JointDescription rootJoint, SDFModel sdfModel)
   {
      List<SDFJoint> sdfJoints = sdfModel.getJoints();
      List<SDFLink> sdfLinks = sdfModel.getLinks();
      List<JointDescription> jointDefinitions = toJointDefinitions(sdfJoints);
      List<LinkDescription> rigidBodyDefinitions = toRigidBodyDefinition(sdfLinks);

      connectKinematics(sdfJoints, rootJoint, jointDefinitions, rigidBodyDefinitions);
      addSensors(sdfLinks, rigidBodyDefinitions);
      correctTransforms(sdfJoints, sdfLinks, jointDefinitions);

      RobotDescription robotDescription = new RobotDescription(sdfModel.getName());
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

   public static void addSensors(List<SDFLink> sdfLinks, List<LinkDescription> rigidBodyDefinitions)
   {
      Map<String, LinkDescription> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                .collect(Collectors.toMap(LinkDescription::getName, Function.identity()));

      for (SDFLink sdfLink : sdfLinks)
      {
         if (sdfLink.getSensors() == null)
            continue;

         LinkDescription linkDescription = rigidBodyDefinitionMap.get(sdfLink.getName());
         JointDescription parentJoint = linkDescription.getParentJoint();

         for (SDFSensor sdfSensor : sdfLink.getSensors())
         {
            List<SensorDescription> sensorDescriptions = toSensorDescription(sdfSensor);
            sensorDescriptions.forEach(parentJoint::addSensor);
         }
      }
   }

   public static void connectKinematics(List<SDFJoint> sdfJoints, JointDescription rootJoint, List<JointDescription> jointDefinitions,
                                        List<LinkDescription> rigidBodyDefinitions)
   {
      if (sdfJoints == null)
         return;

      Map<String, LinkDescription> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                .collect(Collectors.toMap(LinkDescription::getName, Function.identity()));
      Map<String, JointDescription> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));

      Map<String, JointDescription> parentJointDefinitionMap = sdfJoints.stream()
                                                                        .collect(Collectors.toMap(SDFJoint::getChild,
                                                                                                  sdfJoint -> jointDefinitionMap.get(sdfJoint.getName())));

      for (SDFJoint sdfJoint : sdfJoints)
      {
         String parent = sdfJoint.getParent();

         if (parentJointDefinitionMap.containsKey(parent))
            continue;

         // The parent link has no parent joint => it is the root link, we attach it to the given rootJoint.
         LinkDescription rootRigidBody = rigidBodyDefinitionMap.get(parent);
         rootJoint.setLink(rootRigidBody);
         if (rootJoint.getName() == null)
            rootJoint.setName(rootRigidBody.getName());
         parentJointDefinitionMap.put(parent, rootJoint);
         break;
      }

      for (SDFJoint sdfJoint : sdfJoints)
      {
         String parent = sdfJoint.getParent();
         String child = sdfJoint.getChild();
         JointDescription parentJointDescription = parentJointDefinitionMap.get(parent);
         JointDescription jointDefinition = jointDefinitionMap.get(sdfJoint.getName());
         LinkDescription childLinkDescription = rigidBodyDefinitionMap.get(child);

         jointDefinition.setLink(childLinkDescription);
         parentJointDescription.addJoint(jointDefinition);
      }
   }

   private static void correctTransforms(List<SDFJoint> sdfJoints, List<SDFLink> sdfLinks, List<JointDescription> jointDefinitions)
   {
      Map<String, SDFLink> sdfLinkMap = sdfLinks.stream().collect(Collectors.toMap(SDFLink::getName, Function.identity()));
      Map<String, JointDescription> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));
      Map<String, SDFJoint> childToParentJoint = sdfJoints.stream().collect(Collectors.toMap(SDFJoint::getChild, Function.identity()));

      for (SDFJoint sdfJoint : sdfJoints)
      {
         String jointName = sdfJoint.getName();
         JointDescription jointDefinition = jointDefinitionMap.get(jointName);
         LinkDescription childDefinition = jointDefinition.getLink();

         String parentLinkName = sdfJoint.getParent();
         String childLinkName = sdfJoint.getChild();
         SDFJoint parentSDFJoint = childToParentJoint.get(parentLinkName);
         SDFLink parentSDFLink = sdfLinkMap.get(parentLinkName);
         SDFLink childSDFLink = sdfLinkMap.get(childLinkName);

         RigidBodyTransform parentLinkPose = parsePose(parentSDFLink.getPose());
         RigidBodyTransform childLinkPose = parsePose(childSDFLink.getPose());
         RigidBodyTransform parentJointParsedPose = parsePose(parentSDFJoint != null ? parentSDFJoint.getPose() : null);
         RigidBodyTransform jointParsedPose = parsePose(sdfJoint.getPose());

         // Correct joint transform
         RigidBodyTransform transformToParentJoint = jointDefinition.getTransformToParentJoint();
         transformToParentJoint.setAndInvert(parentJointParsedPose);
         transformToParentJoint.multiplyInvertOther(parentLinkPose);
         transformToParentJoint.multiply(childLinkPose);
         transformToParentJoint.multiply(jointParsedPose);
         transformToParentJoint.getRotation().setToZero();
         parentLinkPose.transform(transformToParentJoint.getTranslation());

         // Correct link inertia pose
         RigidBodyTransform inertiaPose = childDefinition.getInertiaPose();
         inertiaPose.prependOrientation(childLinkPose.getRotation());
         inertiaPose.transform(childDefinition.getMomentOfInertia());
         inertiaPose.getRotation().setToZero();

         // Correct visual transform
         for (VisualDescription visualDescription : childDefinition.getLinkGraphics().getVisualDescriptions())
         {
            AffineTransform visualPose = visualDescription.getPose();
            visualPose.prependOrientation(childLinkPose.getRotation());
         }

         for (SensorDescription sensorDescription : jointDefinition.getSensors())
         {
            sensorDescription.getTransformToJoint().prependOrientation(childLinkPose.getRotation());
         }
      }
   }

   public static List<LinkDescription> toRigidBodyDefinition(Collection<SDFLink> sdfLinks)
   {
      if (sdfLinks == null)
         return null;
      else
         return sdfLinks.stream().map(SDFTools::toRigidBodyDefinition).collect(Collectors.toList());
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

   public static List<JointDescription> toJointDefinitions(Collection<SDFJoint> sdfJoints)
   {
      if (sdfJoints == null)
         return null;
      else
         return sdfJoints.stream().map(SDFTools::toJointDefinition).collect(Collectors.toList());
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

   public static PinJointDescription toRevoluteJointDefinition(SDFJoint sdfJoint, boolean ignorePositionLimits)
   {
      PinJointDescription description = new PinJointDescription(sdfJoint.getName());

      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      description.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), description, ignorePositionLimits);
      parseDynamics(sdfJoint.getAxis().getDynamics(), description);

      return description;
   }

   public static SliderJointDescription toPrismaticJointDefinition(SDFJoint sdfJoint)
   {
      SliderJointDescription description = new SliderJointDescription(sdfJoint.getName());
      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      description.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), description, false);
      parseDynamics(sdfJoint.getAxis().getDynamics(), description);

      return description;
   }

   public static FixedJointDescription toFixedJoint(SDFJoint sdfJoint)
   {
      FixedJointDescription description = new FixedJointDescription(sdfJoint.getName());
      RigidBodyTransform parseRigidBodyTransform = parsePose(sdfJoint.getPose());
      description.getTransformToParentJoint().set(parseRigidBodyTransform);
      return description;
   }

   public static FloatingJointDescription toSixDoFJointDefinition(SDFJoint sdfJoint)
   {
      FloatingJointDescription description = new FloatingJointDescription(sdfJoint.getName());
      description.getTransformToParentJoint().set(parsePose(sdfJoint.getPose()));
      return description;
   }

   public static FloatingPlanarJointDescription toPlanarJointDefinition(SDFJoint sdfJoint)
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

   public static List<SensorDescription> toSensorDescription(SDFSensor sdfSensor)
   {
      List<SensorDescription> descriptions = new ArrayList<>();

      switch (sdfSensor.getType())
      {
         case "camera":
         case "multicamera":
         case "depth":
            descriptions.addAll(toCameraSensorDescription(sdfSensor.getCamera()));
            break;
         case "imu":
            descriptions.add(toIMUSensorDescription(sdfSensor.getImu()));
            break;
         case "gpu_ray":
         case "ray":
            descriptions.add(toLidarSensorDescription(sdfSensor.getRay()));
            break;
         default:
            throw new UnsupportedOperationException("Unsupport sensor type: " + sdfSensor.getType());
      }

      int updatePeriod = sdfSensor.getUpdateRate() == null ? -1 : (int) (1000.0 / parseDouble(sdfSensor.getUpdateRate(), 1000.0));

      for (SensorDescription description : descriptions)
      {
         if (description.getName() != null && !description.getName().isEmpty())
            description.setName(sdfSensor.getName() + "_" + description.getName());
         else
            description.setName(sdfSensor.getName());
         description.getTransformToJoint().preMultiply(parsePose(sdfSensor.getPose()));
         description.setUpdatePeriod(updatePeriod);
      }

      return descriptions;
   }

   public static List<CameraSensorDescription> toCameraSensorDescription(List<SDFCamera> sdfCameras)
   {
      return sdfCameras.stream().map(SDFTools::toCameraSensorDescription).collect(Collectors.toList());
   }

   public static CameraSensorDescription toCameraSensorDescription(SDFCamera sdfCamera)
   {
      CameraSensorDescription description = new CameraSensorDescription();
      description.setName(sdfCamera.getName());
      description.getTransformToJoint().set(parsePose(sdfCamera.getPose()));
      description.setFieldOfView(parseDouble(sdfCamera.getHorizontalFov(), Double.NaN));
      description.setClipNear(parseDouble(sdfCamera.getClip().getNear(), Double.NaN));
      description.setClipFar(parseDouble(sdfCamera.getClip().getFar(), Double.NaN));
      description.setImageWidth(parseInteger(sdfCamera.getImage().getWidth(), -1));
      description.setImageHeight(parseInteger(sdfCamera.getImage().getHeight(), -1));
      return description;
   }

   public static LidarSensorDescription toLidarSensorDescription(SDFRay sdfRay)
   {
      LidarSensorDescription description = new LidarSensorDescription();

      SDFRange sdfRange = sdfRay.getRange();
      double sdfRangeMax = parseDouble(sdfRange.getMax(), Double.NaN);
      double sdfRangeMin = parseDouble(sdfRange.getMin(), Double.NaN);
      double sdfRangeResolution = parseDouble(sdfRange.getResolution(), Double.NaN);

      SDFHorizontalScan sdfHorizontalScan = sdfRay.getScan().getHorizontal();
      SDFVerticalScan sdfVerticalScan = sdfRay.getScan().getVertical();
      double maxSweepAngle = parseDouble(sdfHorizontalScan.getMaxAngle(), 0.0);
      double minSweepAngle = parseDouble(sdfHorizontalScan.getMinAngle(), 0.0);
      double maxHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMaxAngle(), 0.0);
      double minHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMinAngle(), 0.0);

      int samples = parseInteger(sdfHorizontalScan.getSamples(), -1) / 3 * 3;
      int scanHeight = sdfVerticalScan == null ? 1 : parseInteger(sdfVerticalScan.getSamples(), 1);

      SDFNoise sdfNoise = sdfRay.getNoise();
      if (sdfNoise != null)
      {
         if ("gaussian".equals(sdfNoise.getType()))
         {
            description.setGaussianNoiseMean(parseDouble(sdfNoise.getMean(), 0.0));
            description.setGaussianNoiseStandardDeviation(parseDouble(sdfNoise.getStddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown noise model: {}.", sdfNoise.getType());
         }
      }

      description.getTransformToJoint().set(parsePose(sdfRay.getPose()));
      description.setPointsPerSweep(samples);
      description.setSweepYawLimits(minSweepAngle, maxSweepAngle);
      description.setHeightPitchLimits(minHeightAngle, maxHeightAngle);
      description.setRangeLimits(sdfRangeMin, sdfRangeMax);
      description.setRangeResolution(sdfRangeResolution);
      description.setScanHeight(scanHeight);
      return description;
   }

   public static IMUSensorDescription toIMUSensorDescription(SDFIMU sdfIMU)
   {
      IMUSensorDescription description = new IMUSensorDescription();

      SDFIMUNoise sdfNoise = sdfIMU.getNoise();
      if (sdfNoise != null)
      {
         if ("gaussian".equals(sdfNoise.getType()))
         {
            SDFNoiseParameters accelerationNoise = sdfNoise.getAccel();
            SDFNoiseParameters angularVelocityNoise = sdfNoise.getRate();

            description.setAccelerationNoiseParameters(parseDouble(accelerationNoise.getMean(), 0.0), parseDouble(accelerationNoise.getStddev(), 0.0));
            description.setAccelerationBiasParameters(parseDouble(accelerationNoise.getBias_mean(), 0.0), parseDouble(accelerationNoise.getBias_stddev(), 0.0));

            description.setAngularVelocityNoiseParameters(parseDouble(angularVelocityNoise.getMean(), 0.0), parseDouble(angularVelocityNoise.getStddev(), 0.0));
            description.setAngularVelocityBiasParameters(parseDouble(angularVelocityNoise.getBias_mean(), 0.0),
                                                         parseDouble(angularVelocityNoise.getBias_stddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown IMU noise model: {}.", sdfNoise.getType());
         }
      }

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
         modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
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

   public static void parseDynamics(SDFDynamics sdfDynamics, OneDoFJointDescription jointDescriptionToParseDynamicsInto)
   {
      double damping = 0.0;
      double stiction = 0.0;

      if (sdfDynamics != null)
      {
         damping = parseDouble(sdfDynamics.getDamping(), 0.0);
         stiction = parseDouble(sdfDynamics.getFriction(), 0.0);
      }

      jointDescriptionToParseDynamicsInto.setDamping(damping);
      jointDescriptionToParseDynamicsInto.setStiction(stiction);
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

   public static int parseInteger(String value, int defaultValue)
   {
      if (value == null)
         return defaultValue;
      return Integer.parseInt(value);
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
