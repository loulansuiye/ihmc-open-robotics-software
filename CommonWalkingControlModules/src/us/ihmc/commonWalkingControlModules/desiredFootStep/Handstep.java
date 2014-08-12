package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.utilities.ArrayTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class Handstep
{
   private static int counter = 0;
   private final String id;
   private final RigidBody endEffector;
   private final PoseReferenceFrame poseReferenceFrame;

   public Handstep(RigidBody endEffector, FramePose framePose)
   {
      this(endEffector, new PoseReferenceFrame("Handstep" + counter, framePose));
   }

   public Handstep(RigidBody endEffector, PoseReferenceFrame poseReferenceFrame)
   {
      this(createAutomaticID(endEffector), endEffector, poseReferenceFrame);
   }

   public Handstep(String id, RigidBody endEffector, PoseReferenceFrame poseReferenceFrame)
   {
      poseReferenceFrame.getParent().checkIsWorldFrame();

      this.id = id;
      this.endEffector = endEffector;
      this.poseReferenceFrame = poseReferenceFrame;
   }

   public Handstep(Handstep handstep)
   {
      this(handstep.endEffector, handstep.poseReferenceFrame);
   }

   private static String createAutomaticID(RigidBody endEffector)
   {
      return endEffector.getName() + "_" + counter++;
   }

   public ReferenceFrame getPoseReferenceFrame()
   {
      return poseReferenceFrame;
   }

   public ReferenceFrame getParentFrame()
   {
      return poseReferenceFrame.getParent();
   }

   public void setX(double x)
   {
      poseReferenceFrame.setX(x);
   }

   public void setY(double y)
   {
      poseReferenceFrame.setY(y);
   }

   public void setZ(double z)
   {
      poseReferenceFrame.setZ(z);
   }

   public void setPose(Handstep newHandstep)
   {
      poseReferenceFrame.setPoseAndUpdate(newHandstep.poseReferenceFrame);
   }

   public void setPose(FramePose newHandstepPose)
   {
      poseReferenceFrame.setPoseAndUpdate(newHandstepPose);
   }

   public void setPose(FramePoint newPosition, FrameOrientation newOrientation)
   {
      poseReferenceFrame.setPoseAndUpdate(newPosition, newOrientation);
   }

   public void setPositionChangeOnlyXY(FramePoint2d position2d)
   {
      poseReferenceFrame.setXYFromPosition2dAndUpdate(position2d);
   }

   public String getId()
   {
      return id;
   }

   public double getX()
   {
      return poseReferenceFrame.getX();
   }

   public double getY()
   {
      return poseReferenceFrame.getY();
   }

   public double getZ()
   {
      return poseReferenceFrame.getZ();
   }

   public double getYaw()
   {
      return poseReferenceFrame.getYaw();
   }

   public double getPitch()
   {
      return poseReferenceFrame.getPitch();
   }

   public double getRoll()
   {
      return poseReferenceFrame.getRoll();
   }

   public void getPose(Point3d pointToPack, Quat4d quaternionToPack)
   {
      poseReferenceFrame.getPose(pointToPack, quaternionToPack);
   }

   public void getPose(Transform3D transformToPack)
   {
      poseReferenceFrame.getPose(transformToPack);
   }

   public void getPose(FramePoint positionToPack, FrameOrientation orientationToPack)
   {
      poseReferenceFrame.getPoseIncludingFrame(positionToPack, orientationToPack);
   }

   public void getPose(FramePose poseToPack)
   {
      poseReferenceFrame.getPoseIncludingFrame(poseToPack);
   }

   public void getPoseReferenceFrameAndUpdate(PoseReferenceFrame poseReferenceFrameToPackAndUpdate)
   {
      poseReferenceFrameToPackAndUpdate.setPoseAndUpdate(poseReferenceFrame);
   }

   public RigidBody getBody()
   {
      return endEffector;
   }

   public void getPosition(Point3d pointToPack)
   {
      poseReferenceFrame.getPosition(pointToPack);
   }

   public void getPositionIncludingFrame(FramePoint framePointToPack)
   {
      poseReferenceFrame.getPositionIncludingFrame(framePointToPack);
   }

   public void getOrientation(Quat4d quaternionToPack)
   {
      poseReferenceFrame.getOrientation(quaternionToPack);
   }

   public void getOrientation(Matrix3d matrixToPack)
   {
      poseReferenceFrame.getOrientation(matrixToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation frameOrientationToPack)
   {
      poseReferenceFrame.getOrientationIncludingFrame(frameOrientationToPack);
   }

   public void getPose2d(FramePose2d framePose2dToPack)
   {
      poseReferenceFrame.getPose2dIncludingFrame(framePose2dToPack);
   }

   public void getPosition2d(FramePoint2d framePoint2dToPack)
   {
      poseReferenceFrame.getPosition2dIncludingFrame(framePoint2dToPack);
   }

   public boolean epsilonEquals(Handstep otherHandstep, double epsilon)
   {
      boolean arePosesEqual = poseReferenceFrame.epsilonEquals(otherHandstep.poseReferenceFrame, epsilon);
      boolean bodiesHaveTheSameName = endEffector.getName().equals(otherHandstep.endEffector.getName());

      return arePosesEqual && bodiesHaveTheSameName;
   }

   public String toString()
   {
      FrameOrientation frameOrientation = new FrameOrientation(poseReferenceFrame);
      frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      double[] ypr = frameOrientation.getYawPitchRoll();
      String yawPitchRoll = "YawPitchRoll = " + ArrayTools.arrayToString(ypr);

      return "id: " + id + " - pose: " + poseReferenceFrame + "\n\tYawPitchRoll= {" + yawPitchRoll + "}";
   }
}
