package us.ihmc.robotDataCommunication;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//    us.ihmc.robotDataCommunication.YoVariableConnectionBurstTest.class
})

public class RobotDataCommunicationBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(RobotDataCommunicationBambooTestSuite.class);
   }
}