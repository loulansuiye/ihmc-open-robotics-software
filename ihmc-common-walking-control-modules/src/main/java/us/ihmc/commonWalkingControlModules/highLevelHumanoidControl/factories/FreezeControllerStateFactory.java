package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class FreezeControllerStateFactory implements HighLevelControllerStateFactory
{
   private FreezeControllerState freezeControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (freezeControllerState == null)
      {
         freezeControllerState = new FreezeControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints(),
                                                           controllerFactoryHelper.getHighLevelControllerParameters(),
                                                           controllerFactoryHelper.getLowLevelControllerOutput());
      }

      return freezeControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.FREEZE_STATE;
   }
}
