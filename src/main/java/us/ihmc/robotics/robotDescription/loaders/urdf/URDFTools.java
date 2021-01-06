package us.ihmc.robotics.robotDescription.loaders.urdf;

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
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.VisualDescription;
import us.ihmc.graphicsDescription.appearance.MaterialDescription;
import us.ihmc.graphicsDescription.appearance.TextureDescription;
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
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFAxis;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFColor;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFDynamics;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFFilenameHolder;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFGazebo;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFGeometry;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFInertia;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFInertial;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFJoint;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFLimit;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFLink;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFMass;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFMaterial;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFModel;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFOrigin;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFCamera;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFIMU;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise.URDFNoiseParameters;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFRay;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFRay.URDFNoise;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFRay.URDFRange;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFRay.URDFScan.URDFHorizontalScan;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFSensor.URDFRay.URDFScan.URDFVerticalScan;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFTexture;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFVisual;
import us.ihmc.robotics.robotDescription.sensors.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.sensors.SensorDescription;

public class URDFTools
{
   private static final Vector3D DEFAULT_ORIGIN_XYZ = new Vector3D();
   private static final Vector3D DEFAULT_ORIGIN_RPY = new Vector3D();

   private static final double DEFAULT_MASS = 0.0;
   private static final double DEFAULT_IXX = 0.0;
   private static final double DEFAULT_IYY = 0.0;
   private static final double DEFAULT_IZZ = 0.0;
   private static final double DEFAULT_IXY = 0.0;
   private static final double DEFAULT_IXZ = 0.0;
   private static final double DEFAULT_IYZ = 0.0;

   private static final Vector3DReadOnly DEFAULT_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private static final double DEFAULT_LOWER_LIMIT = Double.NEGATIVE_INFINITY;
   private static final double DEFAULT_UPPER_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_EFFORT_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_VELOCITY_LIMIT = Double.POSITIVE_INFINITY;

   public static URDFModel loadURDFModel(File urdfFile) throws JAXBException
   {
      return loadURDFModel(urdfFile, Collections.emptyList());
   }

   public static URDFModel loadURDFModel(File urdfFile, Collection<String> resourceDirectories) throws JAXBException
   {
      Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
      File parentFile = urdfFile.getParentFile();

      if (parentFile != null)
      {
         allResourceDirectories.add(parentFile.getAbsolutePath() + File.separator);
         Stream.of(parentFile.listFiles(File::isDirectory)).map(file -> file.getAbsolutePath() + File.separator).forEach(allResourceDirectories::add);
      }

      JAXBContext context = JAXBContext.newInstance(URDFModel.class);
      Unmarshaller um = context.createUnmarshaller();
      URDFModel urdfModel = (URDFModel) um.unmarshal(urdfFile);

      resolvePaths(urdfModel, allResourceDirectories);

      return urdfModel;
   }

   public static URDFModel loadURDFModel(InputStream inputStream, Collection<String> resourceDirectories) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(URDFModel.class);
      Unmarshaller um = context.createUnmarshaller();
      URDFModel urdfModel = (URDFModel) um.unmarshal(inputStream);

      resolvePaths(urdfModel, resourceDirectories);

      return urdfModel;
   }

   public static void resolvePaths(URDFModel urdfModel, Collection<String> resourceDirectories)
   {
      List<URDFFilenameHolder> filenameHolders = urdfModel.getFilenameHolders();

      for (URDFFilenameHolder urdfFilenameHolder : filenameHolders)
      {
         urdfFilenameHolder.setFilename(tryToConvertToPath(urdfFilenameHolder.getFilename(), resourceDirectories));
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
            if (URDFTools.class.getClassLoader().getResource(fullname) != null)
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
         System.err.println("Malformed resource path in URDF file for path: " + filename);
      }

      return null;
   }

   public static RobotDescription toFloatingRootJointRobotDescription(URDFModel urdfModel)
   {
      return toRobotDefinition(new FloatingJointDescription(), urdfModel);
   }

   public static RobotDescription toRobotDefinition(JointDescription rootJoint, URDFModel urdfModel)
   {
      List<URDFLink> urdfLinks = urdfModel.getLinks();
      List<URDFJoint> urdfJoints = urdfModel.getJoints();
      List<URDFGazebo> urdfGazebos = urdfModel.getGazebos();

      List<LinkDescription> rigidBodyDefinitions = toRigidBodyDefinition(urdfLinks);
      List<JointDescription> jointDefinitions = toJointDefinitions(urdfJoints);
      connectKinematics(urdfJoints, rootJoint, rigidBodyDefinitions, jointDefinitions);
      addSensor(urdfGazebos, jointDefinitions);
      simplifyKinematics(rootJoint);
      correctTransforms(rootJoint);

      RobotDescription robotDefinition = new RobotDescription(urdfModel.getName());
      robotDefinition.addRootJoint(rootJoint);

      return robotDefinition;
   }

   public static void addSensor(List<URDFGazebo> urdfGazebos, List<JointDescription> jointDescriptions)
   {
      Map<String, JointDescription> jointDefinitionMap = jointDescriptions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));
      Map<String, JointDescription> linkNameToJointDefinitionMap = jointDescriptions.stream().collect(Collectors.toMap(joint -> joint.getLink().getName(),
                                                                                                                       Function.identity()));

      for (URDFGazebo urdfGazebo : urdfGazebos)
      {
         if (urdfGazebo.getSensor() == null)
            continue;

         List<SensorDescription> sensorDescriptions = toSensorDescription(urdfGazebo.getSensor());
         JointDescription jointDescription = jointDefinitionMap.get(urdfGazebo.getReference());
         if (jointDescription == null)
            jointDescription = linkNameToJointDefinitionMap.get(urdfGazebo.getReference());

         if (jointDescription == null)
         {
            LogTools.error("Could not find reference: " + urdfGazebo.getReference());
            continue;
         }

         if (sensorDescriptions != null)
            sensorDescriptions.forEach(jointDescription::addSensor);
      }
   }

   public static void simplifyKinematics(JointDescription joint)
   {
      List<JointDescription> childrenJoints = new ArrayList<>(joint.getChildrenJoints());

      for (JointDescription child : childrenJoints)
      {
         simplifyKinematics(child);
      }

      JointDescription parentJoint = joint.getParentJoint();
      if (parentJoint == null)
         return;

      if (joint instanceof FixedJointDescription)
      {
         LinkDescription link = joint.getLink();
         RigidBodyTransform transformToParentJoint = joint.getTransformToParentJoint();
         link.applyTransform(transformToParentJoint);
         parentJoint.setLink(merge(parentJoint.getLink().getName(), parentJoint.getLink(), link));

         joint.getKinematicPoints().removeIf(kp ->
         {
            kp.applyTransform(transformToParentJoint);
            parentJoint.addKinematicPoint(kp);
            return true;
         });
         joint.getExternalForcePoints().removeIf(efp ->
         {
            efp.applyTransform(transformToParentJoint);
            parentJoint.addExternalForcePoint(efp);
            return true;
         });
         joint.getGroundContactPoints().removeIf(gcp ->
         {
            gcp.applyTransform(transformToParentJoint);
            parentJoint.addGroundContactPoint(gcp);
            return true;
         });
         joint.getSensors().removeIf(sensor ->
         {
            sensor.applyTransform(transformToParentJoint);
            parentJoint.addSensor(sensor);
            return true;
         });
         joint.getChildrenConstraintDescriptions().removeIf(childConstraint ->
         {
            childConstraint.getTransformToParentJoint().preMultiply(transformToParentJoint);
            parentJoint.addConstraint(childConstraint);
            return true;
         });
         childrenJoints.removeIf(child ->
         {
            child.getTransformToParentJoint().preMultiply(transformToParentJoint);
            parentJoint.addJoint(child);
            return true;
         });
         parentJoint.removeJoint(joint);
      }
   }

   public static LinkDescription merge(String name, LinkDescription linkA, LinkDescription linkB)
   {
      double mergedMass = linkA.getMass() + linkB.getMass();
      Vector3D mergedCoM = new Vector3D();
      mergedCoM.setAndScale(linkA.getMass(), linkA.getCenterOfMassOffset());
      mergedCoM.scaleAdd(linkB.getMass(), linkB.getCenterOfMassOffset(), mergedCoM);
      mergedCoM.scale(1.0 / mergedMass);

      Vector3D translationInertiaA = new Vector3D();
      translationInertiaA.sub(mergedCoM, linkA.getCenterOfMassOffset());
      Matrix3D inertiaA = new Matrix3D(linkA.getMomentOfInertia());
      translateMomentOfInertia(linkA.getMass(), translationInertiaA, inertiaA);

      Vector3D translationInertiaB = new Vector3D();
      translationInertiaB.sub(mergedCoM, linkB.getCenterOfMassOffset());
      Matrix3D inertiaB = new Matrix3D(linkB.getMomentOfInertia());
      translateMomentOfInertia(linkB.getMass(), translationInertiaB, inertiaB);

      Matrix3D mergedInertia = new Matrix3D();
      mergedInertia.add(inertiaA);
      mergedInertia.add(inertiaB);

      LinkDescription merged = new LinkDescription(name);
      merged.setMass(mergedMass);
      merged.setCenterOfMassOffset(mergedCoM);
      merged.setMomentOfInertia(mergedInertia);

      LinkGraphicsDescription mergedGraphics = new LinkGraphicsDescription();
      if (linkA.getLinkGraphics() != null)
         mergedGraphics.combine(linkA.getLinkGraphics());
      if (linkB.getLinkGraphics() != null)
         mergedGraphics.combine(linkB.getLinkGraphics());
      merged.setLinkGraphics(mergedGraphics);

      return merged;
   }

   public static void translateMomentOfInertia(double mass, Tuple3DReadOnly translation, Matrix3DBasics momentOfInertiaToTransform)
   {
      double xp = translation.getX();
      double yp = translation.getY();
      double zp = translation.getZ();

      double xp_xp = xp * xp;
      double yp_yp = yp * yp;
      double zp_zp = zp * zp;

      double txx = mass * (yp_yp + zp_zp);
      double tyy = mass * (xp_xp + zp_zp);
      double tzz = mass * (xp_xp + yp_yp);

      double txy = -mass * xp * yp;
      double txz = -mass * xp * zp;
      double tyz = -mass * yp * zp;

      double m00 = momentOfInertiaToTransform.getM00() + txx;
      double m01 = momentOfInertiaToTransform.getM01() + txy;
      double m02 = momentOfInertiaToTransform.getM02() + txz;
      double m10 = momentOfInertiaToTransform.getM10() + txy;
      double m11 = momentOfInertiaToTransform.getM11() + tyy;
      double m12 = momentOfInertiaToTransform.getM12() + tyz;
      double m20 = momentOfInertiaToTransform.getM20() + txz;
      double m21 = momentOfInertiaToTransform.getM21() + tyz;
      double m22 = momentOfInertiaToTransform.getM22() + tzz;

      momentOfInertiaToTransform.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void connectKinematics(List<URDFJoint> urdfJoints, JointDescription rootJoint, List<LinkDescription> rigidBodyDefinitions,
                                        List<JointDescription> jointDefinitions)
   {
      if (urdfJoints == null)
         return;

      Map<String, LinkDescription> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                .collect(Collectors.toMap(LinkDescription::getName, Function.identity()));
      Map<String, JointDescription> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));

      Map<String, JointDescription> parentJointDefinitionMap = urdfJoints.stream()
                                                                         .collect(Collectors.toMap(urdfJoint -> urdfJoint.getChild().getLink(),
                                                                                                   urdfJoint -> jointDefinitionMap.get(urdfJoint.getName())));

      for (URDFJoint urdfJoint : urdfJoints)
      {
         String parent = urdfJoint.getParent().getLink();

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

      for (URDFJoint urdfJoint : urdfJoints)
      {
         String parent = urdfJoint.getParent().getLink();
         String child = urdfJoint.getChild().getLink();
         JointDescription parentJointDescription = parentJointDefinitionMap.get(parent);
         JointDescription jointDefinition = jointDefinitionMap.get(urdfJoint.getName());
         LinkDescription childLinkDescription = rigidBodyDefinitionMap.get(child);

         jointDefinition.setLink(childLinkDescription);
         parentJointDescription.addJoint(jointDefinition);
      }
   }

   public static void correctTransforms(JointDescription jointDescription)
   {
      RotationMatrixBasics jointRotation = jointDescription.getTransformToParentJoint().getRotation();
      if (jointDescription instanceof OneDoFJointDescription)
         jointRotation.transform(((OneDoFJointDescription) jointDescription).getAxis());
      LinkDescription linkDescription = jointDescription.getLink();
      RigidBodyTransform inertiaPose = linkDescription.getInertiaPose();
      inertiaPose.prependOrientation(jointRotation);
      inertiaPose.transform(linkDescription.getMomentOfInertia());
      inertiaPose.getRotation().setToZero();

      for (SensorDescription sensorDescription : jointDescription.getSensors())
         sensorDescription.getTransformToJoint().prependOrientation(jointRotation);

      for (JointDescription childDescription : jointDescription.getChildrenJoints())
      {
         childDescription.getTransformToParentJoint().prependOrientation(jointRotation);
         correctTransforms(childDescription);
      }

      jointRotation.setToZero();
   }

   public static List<LinkDescription> toRigidBodyDefinition(Collection<URDFLink> urdfLinks)
   {
      if (urdfLinks == null)
         return null;
      else
         return urdfLinks.stream().map(URDFTools::toRigidBodyDefinition).collect(Collectors.toList());
   }

   public static LinkDescription toRigidBodyDefinition(URDFLink urdfLink)
   {
      LinkDescription definition = new LinkDescription(urdfLink.getName());

      URDFInertial urdfInertial = urdfLink.getInertial();

      if (urdfInertial == null)
      {
         definition.setMass(parseMass(null));
         definition.getMomentOfInertia().set(parseMomentOfInertia(null));
         definition.getInertiaPose().set(parseRigidBodyTransform(null));
      }
      else
      {
         definition.setMass(parseMass(urdfInertial.getMass()));
         definition.getMomentOfInertia().set(parseMomentOfInertia(urdfInertial.getInertia()));
         definition.getInertiaPose().set(parseRigidBodyTransform(urdfInertial.getOrigin()));
      }

      if (urdfLink.getVisual() != null)
      {
         LinkGraphicsDescription linkGraphicsDescription = new LinkGraphicsDescription();
         urdfLink.getVisual().stream().map(URDFTools::toVisualDefinition).forEach(linkGraphicsDescription::addVisualDescription);
         definition.setLinkGraphics(linkGraphicsDescription);
      }

      return definition;
   }

   public static List<JointDescription> toJointDefinitions(Collection<URDFJoint> urdfJoints)
   {
      if (urdfJoints == null)
         return null;
      else
         return urdfJoints.stream().map(URDFTools::toJointDefinition).collect(Collectors.toList());
   }

   public static JointDescription toJointDefinition(URDFJoint urdfJoint)
   {
      switch (urdfJoint.getType())
      {
         case "continuous":
            return toRevoluteJointDefinition(urdfJoint, true);
         case "revolute":
            return toRevoluteJointDefinition(urdfJoint, false);
         case "prismatic":
            return toPrismaticJointDefinition(urdfJoint);
         case "fixed":
            return toFixedJoint(urdfJoint);
         case "floating":
            return toSixDoFJointDefinition(urdfJoint);
         case "planar":
            return toPlanarJointDefinition(urdfJoint);
         default:
            throw new RuntimeException("Unexpected value for the joint type: " + urdfJoint.getType());
      }
   }

   public static PinJointDescription toRevoluteJointDefinition(URDFJoint urdfJoint, boolean ignorePositionLimits)
   {
      PinJointDescription definition = new PinJointDescription(urdfJoint.getName());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, ignorePositionLimits);
      parseDynamics(urdfJoint.getDynamics(), definition);
      return definition;
   }

   public static SliderJointDescription toPrismaticJointDefinition(URDFJoint urdfJoint)
   {
      SliderJointDescription definition = new SliderJointDescription(urdfJoint.getName());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, false);
      parseDynamics(urdfJoint.getDynamics(), definition);
      return definition;
   }

   public static FixedJointDescription toFixedJoint(URDFJoint urdfJoint)
   {
      FixedJointDescription definition = new FixedJointDescription(urdfJoint.getName());
      RigidBodyTransform parseRigidBodyTransform = parseRigidBodyTransform(urdfJoint.getOrigin());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform);
      return definition;
   }

   public static FloatingJointDescription toSixDoFJointDefinition(URDFJoint urdfJoint)
   {
      FloatingJointDescription definition = new FloatingJointDescription(urdfJoint.getName());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      return definition;
   }

   public static FloatingPlanarJointDescription toPlanarJointDefinition(URDFJoint urdfJoint)
   {
      Vector3D axis = parseAxis(urdfJoint.getAxis());
      Plane plane;
      if (axis.geometricallyEquals(Axis3D.X, 1.0e-5))
         plane = Plane.YZ;
      else if (axis.geometricallyEquals(Axis3D.Y, 1.0e-5))
         plane = Plane.XZ;
      else if (axis.geometricallyEquals(Axis3D.Z, 1.0e-5))
         plane = Plane.XY;
      else
         throw new IllegalArgumentException("Could determine plane from axis: " + axis);

      FloatingPlanarJointDescription definition = new FloatingPlanarJointDescription(urdfJoint.getName(), plane);
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      return definition;
   }

   public static List<SensorDescription> toSensorDescription(URDFSensor urdfSensor)
   {
      List<SensorDescription> descriptions = new ArrayList<>();

      switch (urdfSensor.getType())
      {
         case "camera":
         case "multicamera":
         case "depth":
            descriptions.addAll(toCameraSensorDescription(urdfSensor.getCamera()));
            break;
         case "imu":
            descriptions.add(toIMUSensorDescription(urdfSensor.getImu()));
            break;
         case "gpu_ray":
         case "ray":
            descriptions.add(toLidarSensorDescription(urdfSensor.getRay()));
            break;
         case "force_torque":
            descriptions.add(new ForceSensorDescription());
            break;
         default:
            LogTools.error("Unsupport sensor type: " + urdfSensor.getType());
            return null;
      }

      int updatePeriod = urdfSensor.getUpdateRate() == null ? -1 : (int) (1000.0 / parseDouble(urdfSensor.getUpdateRate(), 1000.0));

      for (SensorDescription description : descriptions)
      {
         if (description.getName() != null && !description.getName().isEmpty())
            description.setName(urdfSensor.getName() + "_" + description.getName());
         else
            description.setName(urdfSensor.getName());
         description.getTransformToJoint().preMultiply(parsePose(urdfSensor.getPose()));
         description.setUpdatePeriod(updatePeriod);
      }

      return descriptions;
   }

   public static List<CameraSensorDescription> toCameraSensorDescription(List<URDFCamera> urdfCameras)
   {
      return urdfCameras.stream().map(URDFTools::toCameraSensorDescription).collect(Collectors.toList());
   }

   public static CameraSensorDescription toCameraSensorDescription(URDFCamera urdfCamera)
   {
      CameraSensorDescription description = new CameraSensorDescription();
      description.setName(urdfCamera.getName());
      description.getTransformToJoint().set(parsePose(urdfCamera.getPose()));
      description.setFieldOfView(parseDouble(urdfCamera.getHorizontalFov(), Double.NaN));
      description.setClipNear(parseDouble(urdfCamera.getClip().getNear(), Double.NaN));
      description.setClipFar(parseDouble(urdfCamera.getClip().getFar(), Double.NaN));
      description.setImageWidth(parseInteger(urdfCamera.getImage().getWidth(), -1));
      description.setImageHeight(parseInteger(urdfCamera.getImage().getHeight(), -1));
      return description;
   }

   public static LidarSensorDescription toLidarSensorDescription(URDFRay urdfRay)
   {
      LidarSensorDescription description = new LidarSensorDescription();

      URDFRange urdfRange = urdfRay.getRange();
      double urdfRangeMax = parseDouble(urdfRange.getMax(), Double.NaN);
      double urdfRangeMin = parseDouble(urdfRange.getMin(), Double.NaN);
      double urdfRangeResolution = parseDouble(urdfRange.getResolution(), Double.NaN);

      URDFHorizontalScan urdfHorizontalScan = urdfRay.getScan().getHorizontal();
      URDFVerticalScan urdfVerticalScan = urdfRay.getScan().getVertical();
      double maxSweepAngle = parseDouble(urdfHorizontalScan.getMaxAngle(), 0.0);
      double minSweepAngle = parseDouble(urdfHorizontalScan.getMinAngle(), 0.0);
      double maxHeightAngle = urdfVerticalScan == null ? 0.0 : parseDouble(urdfVerticalScan.getMaxAngle(), 0.0);
      double minHeightAngle = urdfVerticalScan == null ? 0.0 : parseDouble(urdfVerticalScan.getMinAngle(), 0.0);

      int samples = parseInteger(urdfHorizontalScan.getSamples(), -1) / 3 * 3;
      int scanHeight = urdfVerticalScan == null ? 1 : parseInteger(urdfVerticalScan.getSamples(), 1);

      URDFNoise urdfNoise = urdfRay.getNoise();
      if (urdfNoise != null)
      {
         if ("gaussian".equals(urdfNoise.getType()))
         {
            description.setGaussianNoiseMean(parseDouble(urdfNoise.getMean(), 0.0));
            description.setGaussianNoiseStandardDeviation(parseDouble(urdfNoise.getStddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown noise model: {}.", urdfNoise.getType());
         }
      }

      description.getTransformToJoint().set(parsePose(urdfRay.getPose()));
      description.setPointsPerSweep(samples);
      description.setSweepYawLimits(minSweepAngle, maxSweepAngle);
      description.setHeightPitchLimits(minHeightAngle, maxHeightAngle);
      description.setRangeLimits(urdfRangeMin, urdfRangeMax);
      description.setRangeResolution(urdfRangeResolution);
      description.setScanHeight(scanHeight);
      return description;
   }

   public static IMUSensorDescription toIMUSensorDescription(URDFIMU urdfIMU)
   {
      IMUSensorDescription description = new IMUSensorDescription();

      URDFIMUNoise urdfNoise = urdfIMU.getNoise();
      if (urdfNoise != null)
      {
         if ("gaussian".equals(urdfNoise.getType()))
         {
            URDFNoiseParameters accelerationNoise = urdfNoise.getAccel();
            URDFNoiseParameters angularVelocityNoise = urdfNoise.getRate();

            description.setAccelerationNoiseParameters(parseDouble(accelerationNoise.getMean(), 0.0), parseDouble(accelerationNoise.getStddev(), 0.0));
            description.setAccelerationBiasParameters(parseDouble(accelerationNoise.getBias_mean(), 0.0), parseDouble(accelerationNoise.getBias_stddev(), 0.0));

            description.setAngularVelocityNoiseParameters(parseDouble(angularVelocityNoise.getMean(), 0.0), parseDouble(angularVelocityNoise.getStddev(), 0.0));
            description.setAngularVelocityBiasParameters(parseDouble(angularVelocityNoise.getBias_mean(), 0.0),
                                                         parseDouble(angularVelocityNoise.getBias_stddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown IMU noise model: {}.", urdfNoise.getType());
         }
      }

      return description;
   }

   public static VisualDescription toVisualDefinition(URDFVisual urdfVisual)
   {
      if (urdfVisual == null)
         return null;

      VisualDescription visualDefinition = new VisualDescription();
      visualDefinition.setName(urdfVisual.getName());
      visualDefinition.setPose(new AffineTransform(parseRigidBodyTransform(urdfVisual.getOrigin())));
      visualDefinition.setMaterial(toMaterialDefinition(urdfVisual.getMaterial()));
      visualDefinition.setGeometry(toGeometryDefinition(urdfVisual.getGeometry()));
      return visualDefinition;
   }

   public static GeometryDescription toGeometryDefinition(URDFGeometry urdfGeometry)
   {
      return toGeometryDefinition(urdfGeometry, Collections.emptyList());
   }

   public static GeometryDescription toGeometryDefinition(URDFGeometry urdfGeometry, List<String> resourceDirectories)
   {
      if (urdfGeometry.getBox() != null)
      {
         Box3DDescription boxGeometryDefinition = new Box3DDescription();
         boxGeometryDefinition.setSize(parseVector3D(urdfGeometry.getBox().getSize(), null));
         return boxGeometryDefinition;
      }
      if (urdfGeometry.getCylinder() != null)
      {
         Cylinder3DDescription cylinderGeometryDefinition = new Cylinder3DDescription();
         cylinderGeometryDefinition.setRadius(parseDouble(urdfGeometry.getCylinder().getRadius(), 0.0));
         cylinderGeometryDefinition.setHeight(parseDouble(urdfGeometry.getCylinder().getLength(), 0.0));
         return cylinderGeometryDefinition;
      }
      if (urdfGeometry.getSphere() != null)
      {
         Sphere3DDescription sphereDescription = new Sphere3DDescription();
         sphereDescription.setRadius(parseDouble(urdfGeometry.getSphere().getRadius(), 0.0));
         return sphereDescription;
      }
      if (urdfGeometry.getMesh() != null)
      {
         ModelFileGeometryDescription modelFileGeometryDefinition = new ModelFileGeometryDescription();
         modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
         modelFileGeometryDefinition.setFileName(urdfGeometry.getMesh().getFilename());
         modelFileGeometryDefinition.setScale(parseVector3D(urdfGeometry.getMesh().getScale(), new Vector3D(1, 1, 1)));
         return modelFileGeometryDefinition;
      }
      throw new IllegalArgumentException("The given URDF Geometry is empty.");
   }

   public static MaterialDescription toMaterialDefinition(URDFMaterial urdfMaterial)
   {
      if (urdfMaterial == null)
         return null;

      MaterialDescription materialDefinition = new MaterialDescription();
      materialDefinition.setName(urdfMaterial.getName());
      materialDefinition.setDiffuseColor(toColorDefinition(urdfMaterial.getColor()));
      materialDefinition.setDiffuseMap(toTextureDefinition(urdfMaterial.getTexture()));
      return materialDefinition;
   }

   public static TextureDescription toTextureDefinition(URDFTexture urdfTexture)
   {
      if (urdfTexture == null)
         return null;

      TextureDescription textureDefinition = new TextureDescription();
      textureDefinition.setTextureFilename(urdfTexture.getFilename());
      return textureDefinition;
   }

   public static ColorDescription toColorDefinition(URDFColor urdfColor)
   {
      if (urdfColor == null)
         return null;

      return ColorDescription.rgba(parseArray(urdfColor.getRGBA(), null));
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

   public static RigidBodyTransform parseRigidBodyTransform(URDFOrigin origin)
   {
      if (origin == null)
         origin = new URDFOrigin();

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.getTranslation().set(parseVector3D(origin.getXYZ(), DEFAULT_ORIGIN_XYZ));
      rigidBodyTransform.getRotation().setEuler(parseVector3D(origin.getRPY(), DEFAULT_ORIGIN_RPY));
      return rigidBodyTransform;
   }

   public static Matrix3D parseMomentOfInertia(URDFInertia inertia)
   {
      if (inertia == null)
         inertia = new URDFInertia();

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

   public static double parseMass(URDFMass urdfMass)
   {
      if (urdfMass == null)
         return DEFAULT_MASS;
      return parseDouble(urdfMass.getValue(), DEFAULT_MASS);
   }

   public static void parseLimit(URDFLimit urdfLimit, OneDoFJointDescription jointDescriptionToParseLimitInto, boolean ignorePositionLimits)
   {
      jointDescriptionToParseLimitInto.setPositionLimits(DEFAULT_LOWER_LIMIT, DEFAULT_UPPER_LIMIT);
      jointDescriptionToParseLimitInto.setEffortLimits(DEFAULT_EFFORT_LIMIT);
      jointDescriptionToParseLimitInto.setVelocityLimits(DEFAULT_VELOCITY_LIMIT);

      if (urdfLimit != null)
      {
         if (!ignorePositionLimits)
         {
            double positionLowerLimit = parseDouble(urdfLimit.getLower(), DEFAULT_LOWER_LIMIT);
            double positionUpperLimit = parseDouble(urdfLimit.getUpper(), DEFAULT_UPPER_LIMIT);
            if (positionLowerLimit < positionUpperLimit)
               jointDescriptionToParseLimitInto.setPositionLimits(positionLowerLimit, positionUpperLimit);
         }
         double effortLimit = parseDouble(urdfLimit.getEffort(), DEFAULT_EFFORT_LIMIT);
         if (Double.isFinite(effortLimit) && effortLimit >= 0)
            jointDescriptionToParseLimitInto.setEffortLimits(effortLimit);
         double velocityLimit = parseDouble(urdfLimit.getVelocity(), DEFAULT_VELOCITY_LIMIT);
         if (Double.isFinite(velocityLimit) && velocityLimit >= 0)
            jointDescriptionToParseLimitInto.setVelocityLimits(velocityLimit);
      }
   }

   public static void parseDynamics(URDFDynamics urdfDynamics, OneDoFJointDescription jointDescriptionToParseDynamicsInto)
   {
      double damping = 0.0;
      double stiction = 0.0;

      if (urdfDynamics != null)
      {
         damping = parseDouble(urdfDynamics.getDamping(), 0.0);
         stiction = parseDouble(urdfDynamics.getFriction(), 0.0);
      }

      jointDescriptionToParseDynamicsInto.setDamping(damping);
      jointDescriptionToParseDynamicsInto.setStiction(stiction);
   }

   public static Vector3D parseAxis(URDFAxis axis)
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
