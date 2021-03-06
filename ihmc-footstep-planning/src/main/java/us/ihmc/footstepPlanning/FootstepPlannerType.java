package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum FootstepPlannerType
{
   PLANAR_REGION_BIPEDAL, PLAN_THEN_SNAP, A_STAR, SIMPLE_BODY_PATH, VIS_GRAPH_WITH_A_STAR;

   public static final FootstepPlannerType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerType generateRandomPlannerType(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public static FootstepPlannerType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }

   public static FootstepPlannerType fromString(String name)
   {
      for (FootstepPlannerType value : values)
      {
         if (name.equals(value.name()))
            return value;
      }

      throw new IllegalArgumentException("Unable to find a corresponding FootstepPlannerType to string : " + name);
   }

}
