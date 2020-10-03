package us.ihmc.robotics.robotDescription.collision;

import gnu.trove.map.hash.TObjectLongHashMap;

/**
 * Helper class that can be used to simplify the generation of collision group ids which can be used
 * with {@link CollisionMaskHolder}s.
 * <p>
 * Each group id is associated with a name to facilitate creation and retrieval of a group.
 * </p>
 */
public class CollisionGroupIdCalculator
{
   private static final long EMPTY_VALUE = -1;
   private long nextId = 0b1;
   private final TObjectLongHashMap<String> registeredCollisionGroupIds = new TObjectLongHashMap<>(64, 1.0f, EMPTY_VALUE);

   /**
    * Creates a new calculator that can generate up to 64 distinct group ids.
    */
   public CollisionGroupIdCalculator()
   {
   }

   /**
    * Creates or retrieves a group id that is associated with the given name.
    * <p>
    * Only 64 ids be created, check {@link #canRegisterNewGroup()} to verify at any time whether a id
    * can be created.
    * </p>
    * 
    * @param groupName name of the group to get the id of.
    * @return the value of the group id.
    */
   public long getGroupId(String groupName)
   {
      long group = registeredCollisionGroupIds.get(groupName);
      if (group == EMPTY_VALUE)
         return nextGroupId(groupName);
      else
         return group;
   }

   private long nextGroupId(String groupName)
   {
      if (!canRegisterNewGroup())
         throw new RuntimeException("Max capacity reached.");

      long groupId = nextId;
      registeredCollisionGroupIds.put(groupName, groupId);
      nextId = shiftBitLeft(nextId);
      return groupId;
   }

   /**
    * Tests if another collision group id can be generated.
    * <p>
    * Only 64 collision group ids can be created.
    * </p>
    * 
    * @return {@code true} if another group id can be created.
    */
   public boolean canRegisterNewGroup()
   {
      return nextId != 0;
   }

   private static long shiftBitLeft(long value)
   {
      return value << 1;
   }
}
