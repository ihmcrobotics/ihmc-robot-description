package us.ihmc.robotics.robotDescription.collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * A collision group is a collection of collidables that represent a category of objects or objects
 * that are part of the same system like a robot system.
 * <p>
 * The use of groups helps simplifying the configuration of possible collisions between collidables.
 * For instance, each collidable can be configured to be allowed to collide with other collidables
 * that are part of a group. Also by configuring two groups as allowed to collide with each other,
 * all the collidables registered in either group is configured to collide with any collidable that
 * is part of the other group.
 * </p>
 */
public class CollisionGroup
{
   /** Representative name for this group. */
   private final String name;
   /** The identifier as a long bit for this group. */
   private final long id;
   /**
    * The list of collidables that are part of this group. A collidable can be part of more than one
    * group.
    */
   private final List<CollisionMaskHolder> collidables = new ArrayList<>();

   /**
    * Creates a new collision group given a representative name and unique long bit.
    * 
    * @param name representative name for this group. Can be used to retrieve this group.
    * @param id   the unique long bit for this group that will be eventually be used when filtering
    *             collisions between collidables.
    */
   public CollisionGroup(String name, long id)
   {
      this.name = name;
      this.id = id;
   }

   /**
    * Adds the given collidable to this group.
    * <p>
    * This operation modifies collidable's collision groups info and leaves the collision mask of the
    * collidable unmodified.
    * </p>
    * 
    * @param collidable the collidable to add to this group.
    */
   public void add(CollisionMaskHolder collidable)
   {
      collidables.add(collidable);
      addToGroup(id, collidable);
   }

   /**
    * Adds the given collidables to this group.
    * <p>
    * This operation modifies collidable's collision groups info and leaves the collision mask of the
    * collidable unmodified.
    * </p>
    * 
    * @param collidables the collidables to add to this group.
    */
   public void addAll(Collection<? extends CollisionMaskHolder> collidables)
   {
      for (CollisionMaskHolder collidable : collidables)
      {
         add(collidable);
      }
   }

   /**
    * Adds the given collidables to this group.
    * <p>
    * This operation modifies the collision groups info and leaves the collision mask unmodified for
    * each collidable.
    * </p>
    * 
    * @param collidables the collidables to add to this group.
    */
   public void addAll(CollisionMaskHolder... collidables)
   {
      for (CollisionMaskHolder collidable : collidables)
      {
         add(collidable);
      }
   }

   /**
    * Removes the given collidable from this group.
    * <p>
    * This operation modifies collidable's collision groups info and leaves the collision mask of the
    * collidable unmodified.
    * </p>
    * 
    * @param collidable the collidable to remove from this group.
    * @return {@code true} if the collidable was successfully removed from this group, {@code false}
    *         otherwise.
    */
   public boolean remove(CollisionMaskHolder collidable)
   {
      boolean modified = collidables.remove(collidable);
      removeFromGroup(id, collidable);
      return modified;
   }

   /**
    * Removes the given collidables from this group.
    * <p>
    * This operation modifies the collision groups info and leaves the collision mask unmodified for
    * each collidable.
    * </p>
    * 
    * @param collidables the collidables to remove from this group.
    * @return {@code true} if at least one collidable was successfully removed from this group,
    *         {@code false} otherwise.
    */
   public boolean removeAll(Collection<? extends CollisionMaskHolder> collidables)
   {
      boolean modified = false;

      for (CollisionMaskHolder collidable : collidables)
      {
         modified |= remove(collidable);
      }

      return modified;
   }

   /**
    * Removes the given collidables from this group.
    * <p>
    * This operation modifies the collision groups info and leaves the collision mask unmodified for
    * each collidable.
    * </p>
    * 
    * @param collidables the collidables to remove from this group.
    * @return {@code true} if at least one collidable was successfully removed from this group,
    *         {@code false} otherwise.
    */
   public boolean removeAll(CollisionMaskHolder... collidables)
   {
      boolean modified = false;

      for (CollisionMaskHolder collidable : collidables)
      {
         modified |= remove(collidable);
      }

      return modified;
   }

   /**
    * Sets the collision mask for all collidables in this group to the given mask.
    * 
    * @param mask the new mask value for all the collidables in this group.
    */
   public void setCollisionMask(long mask)
   {
      for (CollisionMaskHolder collidable : collidables)
      {
         collidable.setCollisionMask(mask);
      }
   }

   /**
    * Adds the given group id to the collision mask of all the collidables in this group.
    * <p>
    * This effectively allows all collidables in this group to collide with the collidables of another
    * group.
    * </p>
    * 
    * @param groupIdToAdd the id to add to the collision mask of the collidables in this group.
    */
   public void addGroupIdToCollisionMask(long groupIdToAdd)
   {
      for (CollisionMaskHolder collidable : collidables)
      {
         collidable.setCollisionMask(addLongBitToMask(groupIdToAdd, collidable.getCollisionMask()));
      }
   }

   /**
    * Removes the given group id from the collision mask of all the collidables in this group.
    * <p>
    * This effectively makes it such that all collidables in this group will not collide with the
    * collidables of another group.
    * </p>
    * 
    * @param groupIdToRemove the id to remove from the collision mask of the collidables in this group.
    */
   public void removeGroupIdFromCollisionMask(long groupIdToRemove)
   {
      for (CollisionMaskHolder collidable : collidables)
      {
         collidable.setCollisionMask(removeLongBitFromMask(groupIdToRemove, collidable.getCollisionMask()));
      }
   }

   /**
    * Sets the collision mask of the collidables in this group such that they cannot collide with
    * anything.
    */
   public void setToCollideWithNothing()
   {
      long mask = 0L;
      setCollisionMask(mask);
   }

   /**
    * Sets the collision mask of the collidables in this group such that they can collide with
    * everything.
    * 
    * @param includeSelfCollision {@code true} if the collidables of this group should be able to
    *                             collide with each other in addition to being to collide with every
    *                             other group.
    */
   public void setToCollideEverything(boolean includeSelfCollision)
   {
      long mask = ~0L;
      if (!includeSelfCollision)
         mask = removeLongBitFromMask(id, mask);
      setCollisionMask(mask);
   }

   /**
    * Adds the given group to the collision mask of all the collidables in this group.
    * <p>
    * This effectively allows all collidables in this group to collide with the collidables of the
    * given group.
    * </p>
    * 
    * @param other the group to add to the collision mask of the collidables in this group.
    */
   public void addGroupToCollideWith(CollisionGroup other)
   {
      addGroupIdToCollisionMask(other.id);
   }

   /**
    * Removes the given group from the collision mask of all the collidables in this group.
    * <p>
    * This effectively makes it such that all collidables in this group will not collide with the
    * collidables of the given group.
    * </p>
    * 
    * @param other the group to remove from the collision mask of the collidables in this group.
    */
   public void removeGroupToCollideWith(CollisionGroup other)
   {
      removeGroupIdFromCollisionMask(other.id);
   }

   /**
    * Returns the name associated with this group.
    * 
    * @return the name associated with this group.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns the id of this group.
    * 
    * @return the id of this group.
    */
   public long getId()
   {
      return id;
   }

   /**
    * Returns the collidables composing this group.
    * 
    * @return the collidables composing this group.
    */
   public List<CollisionMaskHolder> getCollidables()
   {
      return collidables;
   }

   /**
    * Adds the group id to the given collidable.
    * 
    * @param groupToAdd
    * @param collidable
    */
   public static void addToGroup(long groupToAdd, CollisionMaskHolder collidable)
   {
      long newGroupsId = addLongBitToMask(groupToAdd, collidable.getCollisionGroups());
      collidable.setCollisionGroups(newGroupsId);
   }

   public static void removeFromGroup(long groupToRemove, CollisionMaskHolder collidable)
   {
      long newGroupsId = removeLongBitFromMask(groupToRemove, collidable.getCollisionGroups());
      collidable.setCollisionGroups(newGroupsId);
   }

   public static long addLongBitToMask(long longBitToAdd, long maskToModify)
   {
      return maskToModify | longBitToAdd;
   }

   public static long removeLongBitFromMask(long longBitToRemove, long maskToModify)
   {
      return maskToModify & ~longBitToRemove;
   }
}
