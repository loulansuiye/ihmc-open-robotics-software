package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements PacketConsumer<PelvisPosePacket>, PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(new FramePoint(ReferenceFrame.getWorldFrame()));
   private final AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation(
         ReferenceFrame.getWorldFrame()));
   private double trajectoryTime = Double.NaN;

   public DesiredPelvisPoseProvider()
   {
   }

   @Override
   public boolean checkForNewPosition()
   {
      return desiredPelvisPosition.get() != null;
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return desiredPelvisOrientation.get() != null;
   }

   @Override
   public boolean checkForHomePosition()
   {
      return goToHomePosition.getAndSet(false);
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame)
   {
      return desiredPelvisPosition.getAndSet(null);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      FrameOrientation ret = desiredPelvisOrientation.getAndSet(null);

      if (ret == null) return null;

      ret.changeFrame(desiredPelvisFrame);
      return ret;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void receivedPacket(PelvisPosePacket object)
   {
      if (object == null)
         return;

      // If go to home position requested, ignore the other commands.
      if (object.isToHomePosition())
      {
         goToHomePosition.set(true);
         goToHomeOrientation.set(true);
         return;
      }

      if (object.getPosition() != null)
         desiredPelvisPosition.set(new FramePoint(worldFrame, object.getPosition()));
      else
         desiredPelvisPosition.set(null);

      if (object.getOrientation() != null)
         desiredPelvisOrientation.set(new FrameOrientation(worldFrame, object.getOrientation()));
      else
         desiredPelvisOrientation.set(null);

      trajectoryTime = object.getTrajectoryTime();
   }
}
