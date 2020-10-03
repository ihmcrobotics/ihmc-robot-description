package us.ihmc.robotics.robotDescription.collision;

import java.util.ArrayList;
import java.util.LinkedHashMap;

public class OldCollisionMasksHelper
{
   private final LinkedHashMap<String, ArrayList<? extends OldCollisionMaskHolder>> groups = new LinkedHashMap<>();
   private final LinkedHashMap<ArrayList<? extends OldCollisionMaskHolder>, Integer> groupBits = new LinkedHashMap<>();
   private int nextGroupBitMask = 0x01;

   public void addCollisionGroup(String name, ArrayList<? extends OldCollisionMaskHolder> group)
   {
      if (nextGroupBitMask == 0)
      {
         throw new RuntimeException("Number of groups at maximum of 32!");
      }

      groups.put(name, group);
      int groupBit = nextGroupBitMask;
      groupBits.put(group, groupBit);

      addToCollisionGroups(group, groupBit);
      nextGroupBitMask = nextGroupBitMask << 1;
   }

   public int getNextGroupBitMask()
   {
      return nextGroupBitMask;
   }

   public ArrayList<? extends OldCollisionMaskHolder> getCollisionGroup(String name)
   {
      return groups.get(name);
   }

   public void setToCollideWithGroup(String groupOneName, String groupTwoName)
   {
      ArrayList<? extends OldCollisionMaskHolder> groupOne = groups.get(groupOneName);
      ArrayList<? extends OldCollisionMaskHolder> groupTwo = groups.get(groupTwoName);

      Integer bitOne = groupBits.get(groupOne);
      Integer bitTwo = groupBits.get(groupTwo);

      addToCollisionMasks(groupOne, bitTwo);
      addToCollisionMasks(groupTwo, bitOne);
   }

   public void addToCollisionMasks(String name, int collisionMaskToAdd)
   {
      ArrayList<? extends OldCollisionMaskHolder> group = groups.get(name);
      this.addToCollisionMasks(group, collisionMaskToAdd);
   }

   public void setAsSelfCollidingGroup(String name)
   {
      ArrayList<? extends OldCollisionMaskHolder> group = groups.get(name);
      Integer groupBit = groupBits.get(group);

      addToCollisionMasks(group, groupBit);
   }

   public void setAsNonSelfCollidingGroup(String name)
   {
      ArrayList<? extends OldCollisionMaskHolder> group = groups.get(name);
      Integer groupBit = groupBits.get(group);

      removeFromCollisionMasks(group, groupBit);
   }

   private void addToCollisionGroups(ArrayList<? extends OldCollisionMaskHolder> group, int groupBitsToAddToGroupMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         OldCollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionGroup(collisionMaskHolder.getCollisionGroup() | groupBitsToAddToGroupMasks);
      }
   }

   private void addToCollisionMasks(ArrayList<? extends OldCollisionMaskHolder> group, Integer groupBitsToAddToCollisionMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         OldCollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionMask(collisionMaskHolder.getCollisionMask() | groupBitsToAddToCollisionMasks);
      }
   }

   private void removeFromCollisionMasks(ArrayList<? extends OldCollisionMaskHolder> group, Integer groupBitsToRemoveFromCollisionMasks)
   {
      for (int i = 0; i < group.size(); i++)
      {
         OldCollisionMaskHolder collisionMaskHolder = group.get(i);
         collisionMaskHolder.setCollisionMask(collisionMaskHolder.getCollisionMask() & (0xffffffff ^ groupBitsToRemoveFromCollisionMasks));
      }
   }
}
