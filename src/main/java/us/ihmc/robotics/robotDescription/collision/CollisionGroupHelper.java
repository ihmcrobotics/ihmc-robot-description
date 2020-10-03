package us.ihmc.robotics.robotDescription.collision;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * Helper class that can be used to create and manage groups of collidables.
 * <p>
 * Collision groups are to used to facilitate interactions between collidables. Groups can be
 * configured to consider or ignore collisions with another collision group.
 * </p>
 * 
 * @see CollisionGroup
 */
public class CollisionGroupHelper
{
   private final CollisionGroupIdCalculator calculator = new CollisionGroupIdCalculator();
   private final Map<String, CollisionGroup> nameToGroupMap = new HashMap<>();

   /**
    * Creates a new helper that can be used to create up to 64 distinct groups.
    */
   public CollisionGroupHelper()
   {
   }

   /**
    * Tests if a group was registered with the given name.
    * 
    * @param groupName name of the group to test.
    * @return {@code true} if the group was previously registered, {@code false} otherwise.
    */
   public boolean containsCollisionGroup(String groupName)
   {
      return nameToGroupMap.containsKey(groupName);
   }

   /**
    * Creates or retrieves a group by its name.
    * <p>
    * Only 64 groups, check {@link #canRegisterNewGroup()} to verify at any time whether a new
    * collision group can be created.
    * </p>
    * 
    * @param groupName name of the group to retrieve.
    * @return the collision group.
    */
   public CollisionGroup getCollisionGroup(String groupName)
   {
      CollisionGroup group = nameToGroupMap.get(groupName);

      if (group == null)
      {
         group = new CollisionGroup(groupName, calculator.getGroupId(groupName));
         nameToGroupMap.put(groupName, group);
      }

      return group;
   }

   /**
    * Tests if another collision group can be generated.
    * <p>
    * Only 64 collision groups can be created.
    * </p>
    * 
    * @return {@code true} if another group can be created.
    */
   public boolean canRegisterNewGroup()
   {
      return calculator.canRegisterNewGroup();
   }

   /**
    * Adds the given collidable to a group. The group, if already registered, is retrieved by its name,
    * or a new group is created and associated to the given name.
    * <p>
    * This operation modifies collidable's collision groups info and leaves the collision mask of the
    * collidable unmodified.
    * </p>
    * 
    * @param groupName  the name of the group to add the collidable to.
    * @param collidable the collidable to add to the group.
    */
   public void addCollidableToGroup(String groupName, CollisionMaskHolder collidable)
   {
      getCollisionGroup(groupName).add(collidable);
   }

   /**
    * Adds the given collidables to a group. The group, if already registered, is retrieved by its
    * name, or a new group is created and associated to the given name.
    * <p>
    * This operation modifies the collision groups info and leaves the collision mask unmodified for
    * each collidable.
    * </p>
    * 
    * @param groupName   the name of the group to add the collidables to.
    * @param collidables the collidables to add to the group.
    */
   public void addCollidablesToGroup(String groupName, Collection<? extends CollisionMaskHolder> collidables)
   {
      getCollisionGroup(groupName).addAll(collidables);
   }

   /**
    * Removes the given collidable from a group. The group is retrieved by its name if previously
    * registered, if the group was not registered this method does nothing.
    * <p>
    * This operation modifies collidable's collision groups info and leaves the collision mask of the
    * collidable unmodified.
    * </p>
    * 
    * @param groupName  the name of the group to remove the collidable from.
    * @param collidable the collidable to remove from the group.
    * @return {@code true} if the collidable was successfully removed, {@code false} otherwise.
    */
   public boolean removeCollidableFromGroup(String groupName, CollisionMaskHolder collidable)
   {
      if (!containsCollisionGroup(groupName))
         return false;
      else
         return getCollisionGroup(groupName).remove(collidable);
   }

   /**
    * Removes the given collidables from a group. The group is retrieved by its name if previously
    * registered, if the group was not registered this method does nothing.
    * <p>
    * This operation modifies the collision groups info and leaves the collision mask unmodified for
    * each collidable.
    * </p>
    * 
    * @param groupName   the name of the group to remove the collidables from.
    * @param collidables the collidables to remove from the group.
    * @return {@code true} if at least on collidable was successfully removed, {@code false} otherwise.
    */
   public boolean removeCollidablesFromGroup(String groupName, Collection<? extends CollisionMaskHolder> collidables)
   {
      if (!containsCollisionGroup(groupName))
         return false;
      else
         return getCollisionGroup(groupName).removeAll(collidables);
   }

   /**
    * Returns the calculator used internally to generate the group ids.
    * 
    * @return the group id calculator.
    */
   public CollisionGroupIdCalculator getIdCalculator()
   {
      return calculator;
   }
}
