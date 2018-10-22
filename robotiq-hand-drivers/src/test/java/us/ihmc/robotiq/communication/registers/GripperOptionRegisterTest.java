package us.ihmc.robotiq.communication.registers;

import us.ihmc.robotiq.communication.registers.GripperOptionRegister.rICF;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister.rICS;

public class GripperOptionRegisterTest extends RobotiqOutputRegisterTest
{
   @Override
   protected byte getExpectedByteValue()
   {
      return 12;
   }

   @Override
   protected RobotiqOutputRegister getOutputRegister()
   {
      return new GripperOptionRegister(rICF.INDIVIDUAL_FINGER_CONTROL, rICS.INDIVIDUAL_SCISSOR_CONTROL);
   }
}
