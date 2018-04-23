package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.TakeOffState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class TakeOffToFlightCondition implements StateTransitionCondition
{
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final TakeOffState takeOffState;

   public TakeOffToFlightCondition(TakeOffState takeOffState, SideDependentList<FootSwitchInterface> footSwitches)
   {
      this.footSwitches = footSwitches;
      this.takeOffState = takeOffState;
   }

   @Override
   public boolean checkCondition()
   {
      boolean transitionCondition = takeOffState.isDone();
      if (transitionCondition)
      {
         for (RobotSide robotSide : RobotSide.values)
            transitionCondition &= !footSwitches.get(robotSide).hasFootHitGround();
      }
      return transitionCondition;
   }
}