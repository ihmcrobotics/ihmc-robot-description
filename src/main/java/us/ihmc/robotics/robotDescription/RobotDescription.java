package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CylinderCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.SphereCollisionMeshDefinitionData;

public class RobotDescription implements RobotDescriptionNode, GraphicsObjectsHolder
{
   private String name;
   private final List<JointDescription> rootJoints = new ArrayList<>();

   public RobotDescription(String name)
   {
      setName(name);
   }

   public RobotDescription(RobotDescription other)
   {
      name = other.name;
      other.rootJoints.forEach(joint -> rootJoints.add(cloneJointDescriptionRecursive(joint)));
   }

   public void addRootJoint(JointDescription rootJoint)
   {
      rootJoints.add(rootJoint);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public List<JointDescription> getRootJoints()
   {
      return rootJoints;
   }

   @Override
   public List<JointDescription> getChildrenJoints()
   {
      return getRootJoints();
   }

   public JointDescription getJointDescription(String name)
   {
      for (JointDescription rootJoint : rootJoints)
      {
         JointDescription jointDescription = getJointDescriptionRecursively(name, rootJoint);
         if (jointDescription != null)
            return jointDescription;
      }

      return null;
   }

   private static JointDescription getJointDescriptionRecursively(String name, JointDescription jointDescription)
   {
      if (jointDescription.getName().equals(name))
         return jointDescription;

      List<JointDescription> childJointDescriptions = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childJointDescriptions)
      {
         JointDescription jointDescriptionRecursively = getJointDescriptionRecursively(name, childJointDescription);
         if (jointDescriptionRecursively != null)
            return jointDescriptionRecursively;
      }
      return null;
   }

   public JointDescription findJointDescription(Predicate<JointDescription> condition)
   {
      for (JointDescription rootJoint : rootJoints)
      {
         JointDescription result = findJointDescription(condition, rootJoint);
         if (result != null)
            return result;
      }
      return null;
   }

   private static JointDescription findJointDescription(Predicate<JointDescription> condition, JointDescription start)
   {
      if (condition.test(start))
         return start;

      for (JointDescription child : start.getChildrenJoints())
      {
         JointDescription result = findJointDescription(condition, child);
         if (result != null)
            return result;
      }

      return null;
   }

   @Override
   public Graphics3DObject getGraphicsObject(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getLinkGraphics();
   }

   public LinkDescription getLinkDescription(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink();
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower, ignoreInertiaScaleJointList);
   }

   @Override
   public RobotDescription copy()
   {
      return new RobotDescription(this);
   }

   public void addCollisionMeshDefinitionData(CollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder)
   {
      boolean isVisible = collisionMeshDefinitionDataHolder.isVisible();
      List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = collisionMeshDefinitionDataHolder.getCollisionMeshDefinitionData();
      int numberOfDefinitionData = collisionMeshDefinitionDataList.size();

      for (int i = 0; i < numberOfDefinitionData; i++)
      {
         if (collisionMeshDefinitionDataList.get(i) instanceof SphereCollisionMeshDefinitionData)
         {
            addSphereCollisionMeshDefinitionData((SphereCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else if (collisionMeshDefinitionDataList.get(i) instanceof CylinderCollisionMeshDefinitionData)
         {
            addCylinderCollisionMeshDefinitionData((CylinderCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else if (collisionMeshDefinitionDataList.get(i) instanceof BoxCollisionMeshDefinitionData)
         {
            addBoxCollisionMeshDefinitionData((BoxCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else
         {
            throw new IllegalArgumentException("The type of " + getName() + " is not matched among the simple shape Box3D, Sphere3D, Cylinder3D, Capsule3D");
         }
      }
   }

   private void addBoxCollisionMeshDefinitionData(BoxCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCubeReferencedAtCenter(collisionMeshDefinitionData.getLength(),
                                              collisionMeshDefinitionData.getWidth(),
                                              collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.translate(0, 0, -0.5 * collisionMeshDefinitionData.getHeight());
         linkGraphics.addCube(collisionMeshDefinitionData.getLength(),
                              collisionMeshDefinitionData.getWidth(),
                              collisionMeshDefinitionData.getHeight(),
                              collisionMeshDefinitionData.getYoAppearance());
      }
   }

   private void addSphereCollisionMeshDefinitionData(SphereCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addSphere(collisionMeshDefinitionData.getRadius());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addSphere(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getYoAppearance());

      }
   }

   private void addCylinderCollisionMeshDefinitionData(CylinderCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCylinderReferencedAtBottomMiddle(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addCylinder(collisionMeshDefinitionData.getHeight(),
                                  collisionMeshDefinitionData.getRadius(),
                                  collisionMeshDefinitionData.getYoAppearance());
      }
   }

   public static JointDescription cloneJointDescriptionRecursive(JointDescription source)
   {
      List<LoopClosureConstraintDescription> unresolvedConstraintDescriptions = new ArrayList<>();
      JointDescription clone = cloneJointDescriptionRecursive(source, unresolvedConstraintDescriptions);

      for (LoopClosureConstraintDescription constraint : unresolvedConstraintDescriptions)
      {
         JointDescription otherParentJoint = getJointDescriptionRecursively(constraint.getParentJoint().getName(), source);
         LoopClosureConstraintDescription otherConstraint = otherParentJoint.getChildrenConstraintDescriptions().stream()
                                                                            .filter(e -> e.getName().equals(constraint.getName())).findFirst().get();
         String constraintLinkName = otherConstraint.getLink().getName();
         constraint.setLink(findJointDescription(j -> j.getLink().getName().equals(constraintLinkName), clone).getLink());
      }

      return clone;
   }

   private static JointDescription cloneJointDescriptionRecursive(JointDescription source,
                                                                  List<LoopClosureConstraintDescription> unresolvedConstraintDescriptions)
   {
      JointDescription clone = source.copy();

      for (JointDescription child : source.getChildrenJoints())
      {
         JointDescription childClone = cloneJointDescriptionRecursive(child, unresolvedConstraintDescriptions);
         clone.addJoint(childClone);
      }

      if (unresolvedConstraintDescriptions != null)
         unresolvedConstraintDescriptions.addAll(clone.getChildrenConstraintDescriptions());

      return clone;
   }
}
