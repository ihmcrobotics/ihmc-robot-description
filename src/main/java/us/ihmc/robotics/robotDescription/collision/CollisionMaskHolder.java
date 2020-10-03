package us.ihmc.robotics.robotDescription.collision;

/**
 * Implementation of this class, let's call such implementation a collidable, can be used for
 * efficiently filtering possible collisions by defining groups and their possible interactions.
 * <p>
 * A collision group is represented by an id which is typically a long with a single bit set to 1. A
 * collision group can have one or more collidable(s) and a collidable can belong to one or more
 * collision group(s). The group(s) to which a collidable belongs to is saved by activating the bit
 * of each group, in other words the group ids are combined using a bitwise OR operation.
 * <p>
 * Each collidable is given a bit mask which can be used to quickly assess if it can collide with a
 * given group or not by using its id. So in turn, the collision between two collidables can be
 * quickly filtered by testing the mask of one collidable with the groups of the other.
 * </p>
 */
public interface CollisionMaskHolder
{
   /**
    * Returns the long bits representing the collision group(s) to which this collidable belongs.
    * 
    * @return the combined ids of the groups this collidable belongs to.
    */
   long getCollisionGroups();

   /**
    * Sets the combined ids of the groups this collidable belongs to.
    * 
    * @param groups the combined ids of the groups this collidable belongs to.
    */
   void setCollisionGroups(long groups);

   /**
    * Returns the mask associated to this collidable that can be used to test against the collision
    * groups of another collidable.
    * <p>
    * A collision between two collidables can be skipped if
    * {@code (this.getCollisionGroups() & other.getCollisionMask()) == 0x00}.
    * </p>
    * 
    * @return the collision mask for this collidable.
    */
   long getCollisionMask();

   /**
    * Sets the mask for this collidable. The mask can be used to test against the collision groups of
    * another collidable.
    * <p>
    * A collision between two collidables can be skipped if
    * {@code (this.getCollisionGroups() & other.getCollisionMask()) == 0x00}.
    * </p>
    * 
    * @param mask the new mask value.
    */
   void setCollisionMask(long mask);
}
