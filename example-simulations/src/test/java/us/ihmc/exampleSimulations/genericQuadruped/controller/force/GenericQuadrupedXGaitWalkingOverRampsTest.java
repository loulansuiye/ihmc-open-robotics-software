package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import java.io.IOException;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkingOverRampsTest;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;

public class GenericQuadrupedXGaitWalkingOverRampsTest extends QuadrupedXGaitWalkingOverRampsTest
{
   @Override
   public double getDesiredWalkingVelocity()
   {
      return 0.75;
   }

   @Override
   public double getComHeightForRoughTerrain()
   {
      return 0.575;
   }

   @Override
   public QuadrupedInitialPositionParameters getWalkingDownSlopePosition()
   {
      return new InitialWalkDownSlopePosition();
   }

   @Override
   public QuadrupedInitialPositionParameters getWalkingUpSlopePosition()
   {
      return new InitialWalkUpSlopePosition();
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Test
   public void testWalkingOverShallowRamps() throws IOException
   {
      super.testWalkingOverShallowRamps();
   }

   @Tag("slow")
   @Test
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      super.testWalkingOverAggressiveRamps();
   }

   @Test
   public void testWalkingDownSlope() throws IOException
   {
      super.testWalkingDownSlope();
   }

   @Test
   public void testWalkingUpSlope() throws IOException
   {
      super.testWalkingUpSlope();
   }


   private class InitialWalkDownSlopePosition extends GenericQuadrupedDefaultInitialPosition
   {
      @Override
      public Point3D getInitialBodyPosition()
      {
         return new Point3D(0.0, 0.0, 0.05);
      }

      @Override
      public Quaternion getInitialBodyOrientation()
      {
         return new Quaternion(0.0, 0.2, 0.0);
      }
   }

   private class InitialWalkUpSlopePosition extends GenericQuadrupedDefaultInitialPosition
   {
      @Override
      public Point3D getInitialBodyPosition()
      {
         return new Point3D(0.0, 0.0, 0.1);
      }

      @Override
      public Quaternion getInitialBodyOrientation()
      {
         return new Quaternion(0.0, -0.1, 0.0);
      }
   }
}
