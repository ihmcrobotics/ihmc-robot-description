package us.ihmc.robotics.robotDescription.collision;

public interface OldCollisionMaskHolder
{
   public abstract int getCollisionGroup();

   public abstract void setCollisionGroup(int collisionGroup);

   public abstract int getCollisionMask();

   public abstract void setCollisionMask(int collisionMask);
}
