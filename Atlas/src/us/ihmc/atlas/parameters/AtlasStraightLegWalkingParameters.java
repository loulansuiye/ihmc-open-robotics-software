package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;

public class AtlasStraightLegWalkingParameters extends StraightLegWalkingParameters
{
   private final boolean runningOnRealRobot;

   public AtlasStraightLegWalkingParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }
}
