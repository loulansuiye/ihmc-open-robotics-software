package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public abstract class DRCRobotJointMap implements SDFJointNameMap
{
   public abstract String getNameOfJointBeforeChest();

   public abstract String getNameOfJointBeforeThigh(RobotSide robotSide);

   public abstract String getNameOfJointBeforeHand(RobotSide robotSide);

   public abstract SideDependentList<String> getJointBeforeThighNames();

   public abstract String[] getOrderedJointNames();

   public abstract String getHighestNeckPitchJointName();
}