package us.ihmc.robotics.robotDescription.collision;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.geometry.GeometryDescription;

public class CollisionGeometryDescription implements CollisionMaskHolder
{
   private GeometryDescription geometry;
   private RigidBodyTransform transformToParentJoint;

   /** Used to filter which group the geometry will be allowed to collide with. */
   private long collisionMask;
   /** ID representing the collision groups the geometry belongs to. */
   private long collisionGroups;

   public CollisionGeometryDescription()
   {
   }

   public void setGeometry(GeometryDescription geometry)
   {
      this.geometry = geometry;
   }

   public GeometryDescription getGeometry()
   {
      return geometry;
   }

   public void setTransformToParentJoint(RigidBodyTransform transformToParentJoint)
   {
      this.transformToParentJoint = transformToParentJoint;
   }

   @Override
   public void setCollisionMask(long mask)
   {
      collisionMask = mask;
   }

   @Override
   public void setCollisionGroups(long groups)
   {
      collisionGroups = groups;
   }

   public RigidBodyTransform getTransformToParentJoint()
   {
      return transformToParentJoint;
   }

   @Override
   public long getCollisionMask()
   {
      return collisionMask;
   }

   @Override
   public long getCollisionGroups()
   {
      return collisionGroups;
   }
}