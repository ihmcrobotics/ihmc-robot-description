package us.ihmc.robotics.robotDescription.loaders.urdf;

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
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.VisualDescription;
import us.ihmc.graphicsDescription.appearance.MaterialDescription;
import us.ihmc.graphicsDescription.appearance.TextureDescription;
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
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFAxis;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFColor;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFFilenameHolder;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFGeometry;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFInertia;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFInertial;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFJoint;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFLimit;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFLink;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFLinkReference;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFMass;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFMaterial;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFModel;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFOrigin;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFTexture;
import us.ihmc.robotics.robotDescription.loaders.urdf.items.URDFVisual;

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

   public static RobotDescription toFloatingRootJointRobotDescription(RobotDescription robotDefinition, URDFModel urdfModel)
   {
      return toRobotDefinition(new FloatingJointDescription(), urdfModel);
   }

   public static RobotDescription toRobotDefinition(JointDescription rootJoint, URDFModel urdfModel)
   {
      List<URDFLink> urdfLinks = urdfModel.getLinks();
      List<URDFJoint> urdfJoints = urdfModel.getJoints();

      List<LinkDescription> rigidBodyDefinitions = urdfLinks.stream().map(URDFTools::toRigidBodyDefinition).collect(Collectors.toList());
      List<JointDescription> jointDefinitions;
      if (urdfJoints == null)
         jointDefinitions = Collections.emptyList();
      else
         jointDefinitions = urdfJoints.stream().map(URDFTools::toJointDefinition).collect(Collectors.toList());
      LinkDescription rootBodyDefinition = connectKinematics(rigidBodyDefinitions, jointDefinitions, urdfJoints);

      if (rootJoint.getName() == null)
         rootJoint.setName(rootBodyDefinition.getName());
      rootJoint.setLink(rootBodyDefinition);

      RobotDescription robotDefinition = new RobotDescription(urdfModel.getName());
      robotDefinition.addRootJoint(rootJoint);

      return robotDefinition;
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

   public static LinkDescription connectKinematics(List<LinkDescription> rigidBodyDefinitions, List<JointDescription> jointDefinitions,
                                                   List<URDFJoint> urdfJoints)
   {
      Map<String, LinkDescription> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                .collect(Collectors.toMap(LinkDescription::getName, Function.identity()));
      Map<String, JointDescription> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDescription::getName, Function.identity()));

      if (urdfJoints != null)
      {
         Map<String, JointDescription> parentJointDefinitionMap = urdfJoints.stream()
                                                                            .collect(Collectors.toMap(urdfJoint -> urdfJoint.getChild().getLink(),
                                                                                                      urdfJoint -> jointDefinitionMap.get(urdfJoint.getName())));

         for (URDFJoint urdfJoint : urdfJoints)
         {
            URDFLinkReference parent = urdfJoint.getParent();
            URDFLinkReference child = urdfJoint.getChild();
            JointDescription parentJoint = parentJointDefinitionMap.get(parent.getLink());
            LinkDescription childRigidBodyDefinition = rigidBodyDefinitionMap.get(child.getLink());
            JointDescription jointDefinition = jointDefinitionMap.get(urdfJoint.getName());

            jointDefinition.setLink(childRigidBodyDefinition);
            parentJoint.getChildrenJoints().add(jointDefinition);
         }
      }

      if (urdfJoints == null)
      {
         return rigidBodyDefinitions.get(0);
      }
      else
      {
         Map<String, URDFJoint> childToParentJoint = urdfJoints.stream()
                                                               .collect(Collectors.toMap(urdfJoint -> urdfJoint.getChild().getLink(), Function.identity()));

         String rootBodyName = urdfJoints.iterator().next().getParent().getLink();
         URDFJoint parentJoint = childToParentJoint.get(rootBodyName);

         while (parentJoint != null)
         {
            rootBodyName = parentJoint.getParent().getLink();
            parentJoint = childToParentJoint.get(rootBodyName);
         }

         return rigidBodyDefinitionMap.get(rootBodyName);
      }
   }

   public static PinJointDescription toRevoluteJointDefinition(URDFJoint urdfJoint, boolean ignorePositionLimits)
   {
      PinJointDescription definition = new PinJointDescription(urdfJoint.getName());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, ignorePositionLimits);
      return definition;
   }

   public static SliderJointDescription toPrismaticJointDefinition(URDFJoint urdfJoint)
   {
      SliderJointDescription definition = new SliderJointDescription(urdfJoint.getName());
      definition.getTransformToParentJoint().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, false);
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
         modelFileGeometryDefinition.getResourceDirectories().addAll(resourceDirectories);
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

   public static void parseLimit(URDFLimit urdfLimit, OneDoFJointDescription jointDefinitionToParseLimitInto, boolean ignorePositionLimits)
   {
      double lowerLimit, upperLimit, effortLimit, velocityLimit;

      if (urdfLimit != null)
      {
         if (ignorePositionLimits)
         {
            lowerLimit = DEFAULT_LOWER_LIMIT;
            upperLimit = DEFAULT_UPPER_LIMIT;
         }
         else
         {
            lowerLimit = parseDouble(urdfLimit.getLower(), DEFAULT_LOWER_LIMIT);
            upperLimit = parseDouble(urdfLimit.getUpper(), DEFAULT_UPPER_LIMIT);
         }
         effortLimit = parseDouble(urdfLimit.getEffort(), DEFAULT_EFFORT_LIMIT);
         velocityLimit = parseDouble(urdfLimit.getVelocity(), DEFAULT_VELOCITY_LIMIT);
      }
      else
      {
         lowerLimit = DEFAULT_LOWER_LIMIT;
         upperLimit = DEFAULT_UPPER_LIMIT;
         effortLimit = DEFAULT_EFFORT_LIMIT;
         velocityLimit = DEFAULT_VELOCITY_LIMIT;
      }

      jointDefinitionToParseLimitInto.setPositionLimits(lowerLimit, upperLimit);
      jointDefinitionToParseLimitInto.setEffortLimits(effortLimit);
      jointDefinitionToParseLimitInto.setVelocityLimits(velocityLimit);
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
