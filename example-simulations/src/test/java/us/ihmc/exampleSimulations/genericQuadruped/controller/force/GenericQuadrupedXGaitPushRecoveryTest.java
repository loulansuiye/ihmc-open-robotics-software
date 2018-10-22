package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitPushRecoveryTest;

public class GenericQuadrupedXGaitPushRecoveryTest extends QuadrupedXGaitPushRecoveryTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   public double getWalkingSpeed()
   {
      return 0.8;
   }

   @Override
   public double getStepDuration()
   {
      return 0.35;
   }

   @Test
   public void testWalkingForwardFastWithPush()
   {
      super.testWalkingForwardFastWithPush();
   }

   @Tag("slow")
   @Test
   public void testScriptedWalkingForwardFastWithPush()
   {
      super.testScriptedWalkingForwardFastWithPush();
   }

}
