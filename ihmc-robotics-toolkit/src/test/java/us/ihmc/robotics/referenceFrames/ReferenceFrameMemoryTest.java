package us.ihmc.robotics.referenceFrames;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.tools.MemoryTools;

public class ReferenceFrameMemoryTest
{

   @Test
   public void testGarbageCollectionInBroadTrees()
   {
      int beforeMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: before");

      ReferenceFrame world = ReferenceFrame.getWorldFrame();

      List<ReferenceFrame> testFrames = new ArrayList<ReferenceFrame>();
      for (int i = 0; i < 100000; i++)
      {
         PoseReferenceFrame testFrame = new PoseReferenceFrame("test_" + i, world);
         testFrames.add(testFrame);
      }

      ReferenceFrame.getWorldFrame().clearChildren();

      try
      {
         Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
      }

      int duringMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMBWithoutGarbageCollecting("ReferenceFrameTest: during");

      assertTrue(duringMemoryInMB - beforeMemoryInMB > 20);

      int afterMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: after");
      assertTrue("afterMemoryInMB - beforeMemoryInMB = " + (afterMemoryInMB - beforeMemoryInMB), afterMemoryInMB - beforeMemoryInMB < 140);

      testFrames = null;
      afterMemoryInMB = MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("ReferenceFrameTest: after testFrames = null");
      assertTrue(afterMemoryInMB - beforeMemoryInMB < 10);

   }
}
