package us.ihmc.exampleSimulations.groundTruthinator;

import javax.vecmath.Point3d;

public class GroundTruthinatorSensor
{
   private final Point3d sensorPositionInWorldFrame = new Point3d();
   private final Point3d attachmentPositionInRobotFrame = new Point3d();
   private double cableLength;
   
   public GroundTruthinatorSensor(Point3d sensorPosition, Point3d attachmentPosition)
   {
      sensorPositionInWorldFrame.set(sensorPosition);
      attachmentPositionInRobotFrame.set(attachmentPosition);
   }
   
   public void setSensorPositionInWorldFrame(Point3d sensorPositionInWorld)
   {
      this.sensorPositionInWorldFrame.set(sensorPositionInWorld);
   }
   
   public void setAttachmentPositionInRobotFrame(Point3d attachmentPositionInRobotFrame)
   {
      this.attachmentPositionInRobotFrame.set(attachmentPositionInRobotFrame);
   }
   
   public void setCableLength(double cableLength)
   {
      this.cableLength = cableLength;
   }
   
   public void getSensorPositionInWorldFrame(Point3d sensorPositionInWorldToPack)
   {
      sensorPositionInWorldToPack.set(sensorPositionInWorldFrame);
   }
   
   public void getAttachmentPositionInRobotFrame(Point3d attachmentPositionInRobotFrameToPack)
   {
      attachmentPositionInRobotFrameToPack.set(attachmentPositionInRobotFrame);
   }
   
   public double getCableLength()
   {
      return cableLength;
   }
   
   
}
