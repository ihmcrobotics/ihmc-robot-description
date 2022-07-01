package us.ihmc.robotics.robotDescription;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class RobotDescriptionTest
{

   @Test // timeout = 30000
   public void testRobotDescriptionOne()
   {
      RobotDescription robotDescription = new RobotDescription("Test");
      assertEquals("Test", robotDescription.getName());

      robotDescription.setName("TestTwo");
      assertEquals("TestTwo", robotDescription.getName());

      FloatingJointDescription rootJointOne = new FloatingJointDescription("rootJointOne");
      assertEquals("rootJointOne", rootJointOne.getName());
      LinkDescription rootLinkOne = new LinkDescription("rootLinkOne");
      assertEquals("rootLinkOne", rootLinkOne.getName());

      rootLinkOne.setMass(1.2);
      rootLinkOne.setCenterOfMassOffset(new Vector3D(1.0, 2.0, 3.0));
      rootLinkOne.setMomentOfInertia(0.1, 0.2, 0.3);

      assertEquals(1.2, rootLinkOne.getMass(), 1e-7);

      Vector3D comOffsetCheck = new Vector3D();
      rootLinkOne.getCenterOfMassOffset(comOffsetCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(1.0, 2.0, 3.0), comOffsetCheck, 1e-7);
      EuclidCoreTestTools.assertEquals("", new Vector3D(1.0, 2.0, 3.0), rootLinkOne.getCenterOfMassOffset(), 1e-7);

      Matrix3D momentOfInertiaCopy = rootLinkOne.getMomentOfInertiaCopy();
      assertEquals(0.1, momentOfInertiaCopy.getM00(), 1e-7);
      assertEquals(0.2, momentOfInertiaCopy.getM11(), 1e-7);
      assertEquals(0.3, momentOfInertiaCopy.getM22(), 1e-7);

      DMatrixRMaj momentOfInertiaCheck = rootLinkOne.getMomentOfInertia();
      assertEquals(0.1, momentOfInertiaCheck.get(0, 0), 1e-7);
      assertEquals(0.2, momentOfInertiaCheck.get(1, 1), 1e-7);
      assertEquals(0.3, momentOfInertiaCheck.get(2, 2), 1e-7);

      rootJointOne.setLink(rootLinkOne);

      assertTrue(rootJointOne.getLink() == rootLinkOne);

      robotDescription.addRootJoint(rootJointOne);
      List<JointDescription> rootJoints = robotDescription.getRootJoints();

      assertEquals(1, rootJoints.size());
      assertTrue(rootJointOne == rootJoints.get(0));

      PinJointDescription rootJointTwo = new PinJointDescription("rootJointTwo", new Vector3D(-0.1, -0.2, -0.3), Axis3D.Y);
      Vector3D jointAxisCheck = new Vector3D();
      rootJointTwo.getJointAxis(jointAxisCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(0.0, 1.0, 0.0), jointAxisCheck, 1e-7);

      LinkDescription rootLinkTwo = new LinkDescription("rootLinkTwo");
      assertEquals("rootLinkTwo", rootLinkTwo.getName());

      rootLinkTwo.setMass(1.2);
      rootLinkTwo.setCenterOfMassOffset(new Vector3D(1.0, 2.0, 3.0));
      rootLinkTwo.setMomentOfInertia(0.1, 0.2, 0.3);

      rootJointTwo.setLink(rootLinkTwo);

      robotDescription.addRootJoint(rootJointTwo);

      assertEquals(2, robotDescription.getChildrenJoints().size());
      assertTrue(rootJointOne == robotDescription.getChildrenJoints().get(0));
      assertTrue(rootJointTwo == robotDescription.getChildrenJoints().get(1));

      PinJointDescription childJointOne = new PinJointDescription("childJointOne", new Vector3D(1.2, 1.3, 7.7), Axis3D.Z);

      Vector3D jointOffsetCheck = new Vector3D();
      childJointOne.getOffsetFromParentJoint(jointOffsetCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(1.2, 1.3, 7.7), jointOffsetCheck, 1e-7);

      LinkDescription childLinkOne = new LinkDescription("childLinkOne");
      childLinkOne.setMass(3.3);
      DMatrixRMaj childMomentOfInertiaOne = new DMatrixRMaj(new double[][] {{1.0, 0.012, 0.013}, {0.021, 2.0, 0.023}, {0.031, 0.032, 3.0}});
      childLinkOne.setMomentOfInertia(childMomentOfInertiaOne);
      DMatrixRMaj childMomentOfInertiaOneCheck = new DMatrixRMaj(3, 3);
      childLinkOne.getMomentOfInertia(childMomentOfInertiaOneCheck);

      assertTrue(MatrixFeatures_DDRM.isEquals(new DMatrixRMaj(new double[][] {{1.0, 0.012, 0.013}, {0.021, 2.0, 0.023}, {0.031, 0.032, 3.0}}),
                                              childMomentOfInertiaOneCheck,
                                              1e-7));

      rootJointOne.addJoint(childJointOne);

      RigidBodyTransform cameraOneTransformToJoint = new RigidBodyTransform();
      CameraSensorDescription cameraOneDescription = new CameraSensorDescription("cameraOne", cameraOneTransformToJoint);
      childJointOne.addCameraSensor(cameraOneDescription);

      List<CameraSensorDescription> cameraSensors = childJointOne.getCameraSensors();
      assertEquals(1, cameraSensors.size());
      assertTrue(cameraOneDescription == cameraSensors.get(0));

      assertEquals(rootJointOne, childJointOne.getParentJoint());
      assertNull(rootJointOne.getParentJoint());

      childJointOne.setOffsetFromParentJoint(new Vector3D(-0.4, -0.5, -0.6));
      childJointOne.getOffsetFromParentJoint(jointOffsetCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(-0.4, -0.5, -0.6), jointOffsetCheck, 1e-7);

      assertFalse(childJointOne.containsLimitStops());

      double qMin = -0.2;
      double qMax = 0.4;
      double kLimit = 1000.0;
      double bLimit = 100.0;
      childJointOne.setLimitStops(qMin, qMax, kLimit, bLimit);
      double[] limitStopParameters = childJointOne.getLimitStopParameters();
      assertEquals(4, limitStopParameters.length);

      assertEquals(qMin, limitStopParameters[0], 1e-7);
      assertEquals(qMax, limitStopParameters[1], 1e-7);
      assertEquals(kLimit, limitStopParameters[2], 1e-7);
      assertEquals(bLimit, limitStopParameters[3], 1e-7);

      assertEquals(qMin, childJointOne.getLowerLimit(), 1e-7);
      assertEquals(qMax, childJointOne.getUpperLimit(), 1e-7);

      childJointOne.setDamping(4.4);
      assertEquals(4.4, childJointOne.getDamping(), 1e-7);

      childJointOne.setStiction(7.7);
      assertEquals(7.7, childJointOne.getStiction(), 1e-7);

      childJointOne.setEffortLimit(400.3);
      assertEquals(400.3, childJointOne.getEffortLimit(), 1e-7);

      childJointOne.setVelocityLimits(10.0, 30.0);
      assertEquals(10.0, childJointOne.getVelocityLimit(), 1e-7);
      assertEquals(30.0, childJointOne.getVelocityDamping(), 1e-7);

      assertTrue(childJointOne.containsLimitStops());

      childJointOne.getJointAxis(jointAxisCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(0.0, 0.0, 1.0), jointAxisCheck, 1e-7);

      //TODO: Do Axis vectors need to be normalized???
      SliderJointDescription childJointTwo = new SliderJointDescription("childJointTwo", new Vector3D(0.5, 0.7, 0.9), new Vector3D(1.1, 2.2, 3.3));
      assertTrue(Double.POSITIVE_INFINITY == childJointTwo.getEffortLimit());
      assertTrue(Double.POSITIVE_INFINITY == childJointTwo.getVelocityLimit());
      assertFalse(childJointTwo.containsLimitStops());
      assertTrue(Double.NEGATIVE_INFINITY == childJointTwo.getLowerLimit());
      assertTrue(Double.POSITIVE_INFINITY == childJointTwo.getUpperLimit());

      limitStopParameters = childJointTwo.getLimitStopParameters();
      assertTrue(Double.NEGATIVE_INFINITY == limitStopParameters[0]);
      assertTrue(Double.POSITIVE_INFINITY == limitStopParameters[1]);
      assertEquals(0.0, limitStopParameters[2], 1e-7);
      assertEquals(0.0, limitStopParameters[3], 1e-7);

      assertEquals(0.0, childJointTwo.getVelocityDamping(), 1e-7);
      assertEquals(0.0, childJointTwo.getDamping(), 1e-7);
      assertEquals(0.0, childJointTwo.getStiction(), 1e-7);

      childJointTwo.getJointAxis(jointAxisCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(1.1, 2.2, 3.3), jointAxisCheck, 1e-7);

      LinkDescription childLinkTwo = new LinkDescription("childLinkTwo");
      childLinkTwo.setMass(9.9);
      childLinkTwo.setMomentOfInertia(1.9, 2.2, 0.4);

      EuclidCoreTestTools.assertEquals("", new Vector3D(), childLinkTwo.getCenterOfMassOffset(), 1e-7);
      childJointTwo.setLink(childLinkTwo);

      rootJointOne.addJoint(childJointTwo);
      List<JointDescription> childrenJoints = rootJointOne.getChildrenJoints();
      assertEquals(2, childrenJoints.size());

      assertTrue(childJointOne == childrenJoints.get(0));
      assertTrue(childJointTwo == childrenJoints.get(1));

      PinJointDescription childJointThree = new PinJointDescription("childJointThree", new Vector3D(9.9, 0.0, -0.5), Axis3D.X);
      childJointThree.getOffsetFromParentJoint(jointOffsetCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(9.9, 0.0, -0.5), jointOffsetCheck, 1e-7);
      childJointThree.getJointAxis(jointAxisCheck);
      EuclidCoreTestTools.assertEquals("", new Vector3D(1.0, 0.0, 0.0), jointAxisCheck, 1e-7);

      LinkDescription childLinkThree = new LinkDescription("childLinkThree");
      childLinkThree.setMass(1.9);
      childLinkThree.setMomentOfInertia(0.2, 0.3, 0.4);

      LinkGraphicsDescription childGraphicsThree = new LinkGraphicsDescription();
      childLinkThree.setLinkGraphics(childGraphicsThree);

      CollisionMeshDescription childMeshThree = new CollisionMeshDescription();
      childLinkThree.addCollisionMesh(childMeshThree);

      childJointThree.setLink(childLinkThree);

      childJointTwo.addJoint(childJointThree);
      childrenJoints = childJointTwo.getChildrenJoints();
      assertEquals(1, childrenJoints.size());
      assertTrue(childJointThree == childrenJoints.get(0));
      assertTrue(childJointTwo == childJointThree.getParentJoint());

      JointDescription jointDescriptionCheck = robotDescription.getJointDescription("rootJointOne");
      assertTrue(rootJointOne == jointDescriptionCheck);

      jointDescriptionCheck = robotDescription.getJointDescription("rootJointTwo");
      assertTrue(rootJointTwo == jointDescriptionCheck);

      jointDescriptionCheck = robotDescription.getJointDescription("childJointOne");
      assertTrue(childJointOne == jointDescriptionCheck);

      jointDescriptionCheck = robotDescription.getJointDescription("childJointTwo");
      assertTrue(childJointTwo == jointDescriptionCheck);

      jointDescriptionCheck = robotDescription.getJointDescription("childJointThree");
      assertTrue(childJointThree == jointDescriptionCheck);

      assertNull(robotDescription.getJointDescription("noSuchJoint"));
      assertNull(robotDescription.getGraphicsObject("noSuchJoint"));

      Graphics3DObject linkGraphicsCheck = robotDescription.getGraphicsObject("childJointThree");
      assertTrue(linkGraphicsCheck == childGraphicsThree);
   }

   @Test
   public void testCloneConstructor()
   {
      Random random = new Random(457657);

      for (int i = 0; i < 1000; i++)
      {
         String namePrefix = "Bolop";
         int numberOfJoints = random.nextInt(100) + 5;
         RobotDescription original = nextRobotDescription(random, namePrefix, numberOfJoints);
         RobotDescription clone = new RobotDescription(original);
         assertRobotDescriptionEquals(original, clone);
      }
   }

   private static void assertRobotDescriptionEquals(RobotDescription expected, RobotDescription actual)
   {
      if (expected == null && actual == null)
         return;
      if (expected == null || actual == null)
         fail();
      assertEquals(expected.getName(), actual.getName());
      assertEquals(expected.getChildrenJoints().size(), actual.getChildrenJoints().size());

      for (int i = 0; i < expected.getChildrenJoints().size(); i++)
      {
         JointDescription expectedChild = expected.getChildrenJoints().get(i);
         JointDescription actualChild = actual.getChildrenJoints().stream().filter(e -> e.getName().equals(expectedChild.getName())).findFirst().orElse(null);
         assertNotNull(actualChild);
         assertJointDescriptionEqualsRecursive(expectedChild, actualChild);
      }
   }

   private static void assertJointDescriptionEqualsRecursive(JointDescription expected, JointDescription actual)
   {
      assertJointDescriptionEquals(expected, actual);

      for (int i = 0; i < expected.getChildrenJoints().size(); i++)
      {
         JointDescription expectedChild = expected.getChildrenJoints().get(i);
         JointDescription actualChild = actual.getChildrenJoints().stream().filter(e -> e.getName().equals(expectedChild.getName())).findFirst().orElse(null);
         assertNotNull(actualChild);
         assertJointDescriptionEqualsRecursive(expectedChild, actualChild);
      }
   }

   private static void assertJointDescriptionEquals(JointDescription expected, JointDescription actual)
   {
      if (expected == null && actual == null)
         return;
      if (expected == null || actual == null)
         fail();
      try
      {
         assertEquals(expected.getName(), actual.getName());
         assertEquals(expected.getClass(), actual.getClass());
         assertEquals(expected.isDynamic(), actual.isDynamic());
         assertEquals(expected.getOffsetFromParentJoint(), actual.getOffsetFromParentJoint());
         assertLinkDescriptionEquals(expected.getLink(), actual.getLink());

         if (expected instanceof FloatingJointDescription)
            assertFloatingJointDescriptionPropertiesEqual((FloatingJointDescription) expected, (FloatingJointDescription) actual);
         else if (expected instanceof FloatingPlanarJointDescription)
            assertFloatingPlanarJointDescriptionPropertiesEqual((FloatingPlanarJointDescription) expected, (FloatingPlanarJointDescription) actual);
         else if (expected instanceof OneDoFJointDescription)
            assertOneDoFJointDescriptionPropertiesEqual((OneDoFJointDescription) expected, (OneDoFJointDescription) actual);
         else
            fail("Assertions not implemented for: " + expected.getClass().getSimpleName());

         { // Quick assertions on parent joint
            JointDescription expectedParentJoint = expected.getParentJoint();
            JointDescription actualParentJoint = actual.getParentJoint();

            if (expectedParentJoint == null)
            {
               assertNull(actualParentJoint);
            }
            else
            {
               assertNotNull(actualParentJoint);
               assertEquals(expectedParentJoint.getName(), actualParentJoint.getName());
            }
         }

         { // Quick assertions on child joints
            assertEquals(expected.getChildrenJoints().size(), actual.getChildrenJoints().size());

            for (int i = 0; i < expected.getChildrenJoints().size(); i++)
            {
               String childName = expected.getChildrenJoints().get(i).getName();
               assertTrue(actual.getChildrenJoints().stream().anyMatch(actualChild -> actualChild.getName().equals(childName)));
            }
         }

         assertEquals(expected.getKinematicPoints().size(), actual.getKinematicPoints().size());
         for (int i = 0; i < expected.getKinematicPoints().size(); i++)
         {
            KinematicPointDescription expectedSensor = expected.getKinematicPoints().get(i);
            String sensorName = expectedSensor.getName();
            KinematicPointDescription actualSensor = actual.getKinematicPoints().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getOffsetFromJoint(), actualSensor.getOffsetFromJoint());
         }

         assertEquals(expected.getExternalForcePoints().size(), actual.getExternalForcePoints().size());
         for (int i = 0; i < expected.getExternalForcePoints().size(); i++)
         {
            ExternalForcePointDescription expectedSensor = expected.getExternalForcePoints().get(i);
            String sensorName = expectedSensor.getName();
            ExternalForcePointDescription actualSensor = actual.getExternalForcePoints().stream().filter(e -> e.getName().equals(sensorName)).findFirst()
                                                               .orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getOffsetFromJoint(), actualSensor.getOffsetFromJoint());
         }

         assertEquals(expected.getGroundContactPoints().size(), actual.getGroundContactPoints().size());
         for (int i = 0; i < expected.getGroundContactPoints().size(); i++)
         {
            GroundContactPointDescription expectedSensor = expected.getGroundContactPoints().get(i);
            String sensorName = expectedSensor.getName();
            GroundContactPointDescription actualSensor = actual.getGroundContactPoints().stream().filter(e -> e.getName().equals(sensorName)).findFirst()
                                                               .orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getOffsetFromJoint(), actualSensor.getOffsetFromJoint());
         }

         assertEquals(expected.getWrenchSensors().size(), actual.getWrenchSensors().size());
         for (int i = 0; i < expected.getWrenchSensors().size(); i++)
         {
            JointWrenchSensorDescription expectedSensor = expected.getWrenchSensors().get(i);
            String sensorName = expectedSensor.getName();
            JointWrenchSensorDescription actualSensor = actual.getWrenchSensors().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getTransformToJoint(), actualSensor.getTransformToJoint());
         }

         assertEquals(expected.getCameraSensors().size(), actual.getCameraSensors().size());
         for (int i = 0; i < expected.getCameraSensors().size(); i++)
         {
            CameraSensorDescription expectedSensor = expected.getCameraSensors().get(i);
            String sensorName = expectedSensor.getName();
            CameraSensorDescription actualSensor = actual.getCameraSensors().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getFieldOfView(), actualSensor.getFieldOfView());
            assertEquals(expectedSensor.getClipNear(), actualSensor.getClipNear());
            assertEquals(expectedSensor.getClipFar(), actualSensor.getClipFar());
            assertEquals(expectedSensor.getImageWidth(), actualSensor.getImageWidth());
            assertEquals(expectedSensor.getImageHeight(), actualSensor.getImageHeight());
            assertEquals(expectedSensor.getTransformToJoint(), actualSensor.getTransformToJoint());
         }

         assertEquals(expected.getIMUSensors().size(), actual.getIMUSensors().size());
         for (int i = 0; i < expected.getIMUSensors().size(); i++)
         {
            IMUSensorDescription expectedSensor = expected.getIMUSensors().get(i);
            String sensorName = expectedSensor.getName();
            IMUSensorDescription actualSensor = actual.getIMUSensors().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getAccelerationNoiseMean(), actualSensor.getAccelerationNoiseMean());
            assertEquals(expectedSensor.getAccelerationNoiseStandardDeviation(), actualSensor.getAccelerationNoiseStandardDeviation());
            assertEquals(expectedSensor.getAccelerationBiasMean(), actualSensor.getAccelerationBiasMean());
            assertEquals(expectedSensor.getAccelerationBiasStandardDeviation(), actualSensor.getAccelerationBiasStandardDeviation());
            assertEquals(expectedSensor.getAngularVelocityNoiseMean(), actualSensor.getAngularVelocityNoiseMean());
            assertEquals(expectedSensor.getAngularVelocityNoiseStandardDeviation(), actualSensor.getAngularVelocityNoiseStandardDeviation());
            assertEquals(expectedSensor.getAngularVelocityBiasMean(), actualSensor.getAngularVelocityBiasMean());
            assertEquals(expectedSensor.getAngularVelocityBiasStandardDeviation(), actualSensor.getAngularVelocityBiasStandardDeviation());
            assertEquals(expectedSensor.getTransformToJoint(), actualSensor.getTransformToJoint());
         }

         assertEquals(expected.getLidarSensors().size(), actual.getLidarSensors().size());
         for (int i = 0; i < expected.getLidarSensors().size(); i++)
         {
            LidarSensorDescription expectedSensor = expected.getLidarSensors().get(i);
            String sensorName = expectedSensor.getName();
            LidarSensorDescription actualSensor = actual.getLidarSensors().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.getSweepYawMin(), actualSensor.getSweepYawMin());
            assertEquals(expectedSensor.getSweepYawMax(), actualSensor.getSweepYawMax());
            assertEquals(expectedSensor.getHeightPitchMin(), actualSensor.getHeightPitchMin());
            assertEquals(expectedSensor.getHeightPitchMax(), actualSensor.getHeightPitchMax());
            assertEquals(expectedSensor.getMinRange(), actualSensor.getMinRange());
            assertEquals(expectedSensor.getMaxRange(), actualSensor.getMaxRange());
            assertEquals(expectedSensor.getPointsPerSweep(), actualSensor.getPointsPerSweep());
            assertEquals(expectedSensor.getScanHeight(), actualSensor.getScanHeight());
            assertEquals(expectedSensor.getTransformToJoint(), actualSensor.getTransformToJoint());
         }

         assertEquals(expected.getForceSensors().size(), actual.getForceSensors().size());
         for (int i = 0; i < expected.getForceSensors().size(); i++)
         {
            ForceSensorDescription expectedSensor = expected.getForceSensors().get(i);
            String sensorName = expectedSensor.getName();
            ForceSensorDescription actualSensor = actual.getForceSensors().stream().filter(e -> e.getName().equals(sensorName)).findFirst().orElse(null);
            assertNotNull(actualSensor);
            assertEquals(expectedSensor.useShapeCollision(), actualSensor.useShapeCollision());
            assertEquals(expectedSensor.useGroundContactPoints(), actualSensor.useGroundContactPoints());
            assertEquals(expectedSensor.getTransformToJoint(), actualSensor.getTransformToJoint());
         }
      }
      catch (Throwable e)
      {
         throw new AssertionFailedError("Assertion failed for joint: " + expected.getName(), e);
      }
   }

   private static void assertFloatingJointDescriptionPropertiesEqual(FloatingJointDescription expected, FloatingJointDescription actual)
   {
      assertEquals(expected.getJointVariableName(), actual.getJointVariableName());
   }

   private static void assertFloatingPlanarJointDescriptionPropertiesEqual(FloatingPlanarJointDescription expected, FloatingPlanarJointDescription actual)
   {
      assertEquals(expected.getPlane(), actual.getPlane());
   }

   private static void assertOneDoFJointDescriptionPropertiesEqual(OneDoFJointDescription expected, OneDoFJointDescription actual)
   {
      assertEquals(expected.containsLimitStops(), actual.containsLimitStops());
      assertEquals(expected.getLowerLimit(), actual.getLowerLimit());
      assertEquals(expected.getUpperLimit(), actual.getUpperLimit());
      assertArrayEquals(expected.getLimitStopParameters(), actual.getLimitStopParameters());
      assertEquals(expected.getDamping(), actual.getDamping());
      assertEquals(expected.getStiction(), actual.getStiction());
      assertEquals(expected.getVelocityLimit(), actual.getVelocityLimit());
      assertEquals(expected.getVelocityDamping(), actual.getVelocityDamping());
      assertEquals(expected.getJointAxis(), actual.getJointAxis());
      assertEquals(expected.getEffortLimit(), actual.getEffortLimit());
   }

   private static void assertLinkDescriptionEquals(LinkDescription expected, LinkDescription actual)
   {
      if (expected == null && actual == null)
         return;
      if (expected == null || actual == null)
         fail();
      assertEquals(expected.getName(), actual.getName());
      assertEquals(expected.getMass(), actual.getMass());
      assertEquals(expected.getCenterOfMassOffset(), actual.getCenterOfMassOffset());
      assertTrue(MatrixFeatures_DDRM.isEquals(expected.getMomentOfInertia(), actual.getMomentOfInertia()));
      // TODO Implement assertions for LinkGraphicsDescription and CollisionMeshDescription.
   }

   private static RobotDescription nextRobotDescription(Random random, String namePrefix, int numberOfJoints)
   {
      List<JointDescription> joints = nextJointDescriptionTree(random, namePrefix, numberOfJoints);
      addSensors(random, joints);
      RobotDescription next = new RobotDescription(namePrefix + "_robot");
      next.addRootJoint(joints.get(0));
      return next;
   }

   private static void addSensors(Random random, List<? extends JointDescription> jointDescriptions)
   {
      int n = 4;

      for (JointDescription jointDescription : jointDescriptions)
      {
         if (jointDescriptions.size() > 1 && random.nextBoolean())
            continue;

         String jointName = jointDescription.getName();

         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addKinematicPoint(nextKinematicPointDescription(random, jointName + "_kp" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addExternalForcePoint(nextExternalForcePointDescription(random, jointName + "_efp" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addGroundContactPoint(nextGroundContactPointDescription(random, jointName + "_gcp" + i));
         }

         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addJointWrenchSensor(nextJointWrenchSensorDescription(random, jointName + "_jws" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addCameraSensor(nextCameraSensorDescription(random, jointName + "_cam" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addIMUSensor(nextIMUSensorDescription(random, jointName + "_imu" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addLidarSensor(nextLidarSensorDescription(random, jointName + "_lidar" + i));
         }
         if (random.nextBoolean())
         {
            for (int i = 0; i < random.nextInt(n); i++)
               jointDescription.addForceSensor(nextForceSensorDescription(random, jointName + "_fs" + i));
         }
      }
   }

   private static List<JointDescription> nextJointDescriptionTree(Random random, String namePrefix, int numberOfJoints)
   {
      List<JointDescription> jointDescriptions = new ArrayList<>();

      JointDescription parentJoint = null;

      int jointsRemaining = numberOfJoints;
      int branchCounter = 0;

      while (jointsRemaining > 0)
      {
         int branchSize = jointsRemaining == 1 ? 1 : random.nextInt(jointsRemaining - 1) + 1;
         List<JointDescription> branchJoints = nextJointDescriptionChain(random, namePrefix + "_b" + branchCounter, branchSize);
         if (parentJoint != null)
            parentJoint.addJoint(branchJoints.get(0));
         jointDescriptions.addAll(branchJoints);
         parentJoint = jointDescriptions.get(random.nextInt(jointDescriptions.size()));
         jointsRemaining -= branchSize;
         branchCounter++;
      }

      return jointDescriptions;
   }

   private static List<JointDescription> nextJointDescriptionChain(Random random, String namePrefix, int numberOfJoints)
   {
      List<JointDescription> jointDescriptions = new ArrayList<>();

      JointDescription parentJoint = null;

      for (int i = 0; i < numberOfJoints; i++)
      {
         JointDescription joint = nextJointDescription(random, namePrefix + "_j" + i);
         joint.setLink(nextLinkDescription(random, namePrefix + "_l" + i));
         if (parentJoint != null)
            parentJoint.addJoint(joint);
         jointDescriptions.add(joint);
         parentJoint = joint;
      }

      return jointDescriptions;
   }

   private static JointDescription nextJointDescription(Random random, String name)
   {
      switch (random.nextInt(4))
      {
         case 0:
            return nextFloatingJointDescription(random, name);
         case 1:
            return nextFloatingPlanarJointDescription(random, name);
         case 2:
            return nextPinJointDescription(random, name);
         default:
            return nextSliderJointDescription(random, name);
      }
   }

   private static FloatingJointDescription nextFloatingJointDescription(Random random, String name)
   {
      FloatingJointDescription next = new FloatingJointDescription(name, name + "VarName");
      next.setIsDynamic(random.nextBoolean());
      next.setOffsetFromParentJoint(EuclidCoreRandomTools.nextPoint3D(random));
      return next;
   }

   private static FloatingPlanarJointDescription nextFloatingPlanarJointDescription(Random random, String name)
   {
      FloatingPlanarJointDescription next = new FloatingPlanarJointDescription(name, Plane.values[random.nextInt(Plane.values.length)]);
      next.setIsDynamic(random.nextBoolean());
      next.setOffsetFromParentJoint(EuclidCoreRandomTools.nextPoint3D(random));
      return next;
   }

   private static PinJointDescription nextPinJointDescription(Random random, String name)
   {
      PinJointDescription next = new PinJointDescription(name, EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextUnitVector3D(random));
      next.setIsDynamic(random.nextBoolean());
      next.setVelocityLimits(EuclidCoreRandomTools.nextDouble(random), EuclidCoreRandomTools.nextDouble(random));
      next.setDamping(EuclidCoreRandomTools.nextDouble(random));
      if (random.nextBoolean())
         next.setLimitStops(EuclidCoreRandomTools.nextDouble(random, -Math.PI, 0.0),
                            EuclidCoreRandomTools.nextDouble(random, 0, Math.PI),
                            EuclidCoreRandomTools.nextDouble(random),
                            EuclidCoreRandomTools.nextDouble(random));
      next.setEffortLimit(EuclidCoreRandomTools.nextDouble(random));
      return next;
   }

   private static SliderJointDescription nextSliderJointDescription(Random random, String name)
   {
      SliderJointDescription next = new SliderJointDescription(name, EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextUnitVector3D(random));
      next.setIsDynamic(random.nextBoolean());
      next.setVelocityLimits(EuclidCoreRandomTools.nextDouble(random), EuclidCoreRandomTools.nextDouble(random));
      next.setDamping(EuclidCoreRandomTools.nextDouble(random));
      if (random.nextBoolean())
         next.setLimitStops(EuclidCoreRandomTools.nextDouble(random, -Math.PI, 0.0),
                            EuclidCoreRandomTools.nextDouble(random, 0, Math.PI),
                            EuclidCoreRandomTools.nextDouble(random),
                            EuclidCoreRandomTools.nextDouble(random));
      next.setEffortLimit(EuclidCoreRandomTools.nextDouble(random));
      return next;
   }

   private static LinkDescription nextLinkDescription(Random random, String name)
   {
      LinkDescription next = new LinkDescription(name);
      next.setLinkGraphics(nextLinkGraphicsDescription(random));
      next.addCollisionMesh(nextCollisionMeshDescription(random));
      next.setMass(random.nextDouble());
      next.setCenterOfMassOffset(EuclidCoreRandomTools.nextPoint3D(random));
      next.setMomentOfInertia(EuclidCoreRandomTools.nextMatrix3D(random));
      return next;
   }

   private static LinkGraphicsDescription nextLinkGraphicsDescription(Random random)
   {
      LinkGraphicsDescription next = new LinkGraphicsDescription();

      int size = random.nextInt(5);

      for (int i = 0; i < size; i++)
      {
         switch (random.nextInt(9))
         {
            case 0:
               next.addCube(random.nextDouble(), random.nextDouble(), random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 1:
               next.addWedge(random.nextDouble(), random.nextDouble(), random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 2:
               next.addSphere(random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 3:
               next.addCapsule(random.nextDouble(), random.nextDouble() + 2.0, YoAppearance.randomColor(random));
               break;
            case 4:
               next.addEllipsoid(random.nextDouble(), random.nextDouble(), random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 5:
               next.addCylinder(random.nextDouble(), random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 6:
               next.addCone(random.nextDouble(), random.nextDouble(), YoAppearance.randomColor(random));
               break;
            case 7:
               next.addGenTruncatedCone(random.nextDouble(),
                                        random.nextDouble(),
                                        random.nextDouble(),
                                        random.nextDouble(),
                                        random.nextDouble(),
                                        YoAppearance.randomColor(random));
               break;
            default:
               break;
         }
         next.translate(EuclidCoreRandomTools.nextPoint3D(random));
         next.rotate(EuclidCoreRandomTools.nextQuaternion(random));
      }

      return next;
   }

   private static CollisionMeshDescription nextCollisionMeshDescription(Random random)
   {
      CollisionMeshDescription next = new CollisionMeshDescription();
      int size = random.nextInt(5);

      for (int i = 0; i < size; i++)
      {
         switch (random.nextInt(4))
         {
            case 0:
               next.addSphere(random.nextDouble());
               break;
            case 1:
               next.addCapsule(random.nextDouble(), EuclidGeometryRandomTools.nextLineSegment3D(random));
               break;
            case 2:
               next.addCubeReferencedAtBottomMiddle(random.nextDouble(), random.nextDouble(), random.nextDouble());
               break;
            default:
               next.addCylinderReferencedAtBottomMiddle(random.nextDouble(), random.nextDouble());
               break;
         }
      }

      return next;
   }

   private static KinematicPointDescription nextKinematicPointDescription(Random random, String name)
   {
      return new KinematicPointDescription(name, EuclidCoreRandomTools.nextPoint3D(random));
   }

   private static ExternalForcePointDescription nextExternalForcePointDescription(Random random, String name)
   {
      return new ExternalForcePointDescription(name, EuclidCoreRandomTools.nextPoint3D(random));
   }

   private static GroundContactPointDescription nextGroundContactPointDescription(Random random, String name)
   {
      return new GroundContactPointDescription(name, EuclidCoreRandomTools.nextVector3D(random), random.nextInt(20));
   }

   private static JointWrenchSensorDescription nextJointWrenchSensorDescription(Random random, String name)
   {
      return new JointWrenchSensorDescription(name, EuclidCoreRandomTools.nextRigidBodyTransform(random));
   }

   private static CameraSensorDescription nextCameraSensorDescription(Random random, String name)
   {
      CameraSensorDescription next = new CameraSensorDescription(name,
                                                                 EuclidCoreRandomTools.nextRigidBodyTransform(random),
                                                                 EuclidCoreRandomTools.nextDouble(random, 0, 360),
                                                                 EuclidCoreRandomTools.nextDouble(random, 0, 1),
                                                                 EuclidCoreRandomTools.nextDouble(random, 1, 100));
      next.setImageHeight(random.nextInt(1024));
      next.setImageWidth(random.nextInt(1024));
      return next;
   }

   private static IMUSensorDescription nextIMUSensorDescription(Random random, String name)
   {
      IMUSensorDescription next = new IMUSensorDescription(name, EuclidCoreRandomTools.nextRigidBodyTransform(random));
      next.setAccelerationNoiseParameters(random.nextDouble(), random.nextDouble());
      next.setAccelerationBiasParameters(random.nextDouble(), random.nextDouble());
      next.setAngularVelocityNoiseParameters(random.nextDouble(), random.nextDouble());
      next.setAngularVelocityBiasParameters(random.nextDouble(), random.nextDouble());
      return next;
   }

   private static LidarSensorDescription nextLidarSensorDescription(Random random, String name)
   {
      LidarSensorDescription next = new LidarSensorDescription(name, EuclidCoreRandomTools.nextRigidBodyTransform(random));
      next.setSweepYawLimits(EuclidCoreRandomTools.nextDouble(random, -180, 0), EuclidCoreRandomTools.nextDouble(random, 0, 180));
      next.setHeightPitchLimits(EuclidCoreRandomTools.nextDouble(random, -180, 0), EuclidCoreRandomTools.nextDouble(random, 0, 180));
      next.setRangeLimits(EuclidCoreRandomTools.nextDouble(random, 0, 1), EuclidCoreRandomTools.nextDouble(random, 1, 100));
      next.setPointsPerSweep(random.nextInt(100000));
      next.setScanHeight(random.nextInt(10));
      return next;
   }

   private static ForceSensorDescription nextForceSensorDescription(Random random, String name)
   {
      ForceSensorDescription next = new ForceSensorDescription(name, EuclidCoreRandomTools.nextRigidBodyTransform(random));
      next.setUseGroundContactPoints(random.nextBoolean());
      next.setUseShapeCollision(random.nextBoolean());
      return next;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(RobotDescriptionTest.class);
   }

}
