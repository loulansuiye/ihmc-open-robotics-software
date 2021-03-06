package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class InitialClearancePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   public static final double defaultLeaveTimeInPercentOfTrajectoryTime = 0.25;

   private final boolean allowMultipleFrames;

   private final YoVariableRegistry registry;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoDouble leaveTime;
   private final YoPolynomial xyPolynomial, zPolynomial;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames initialDirection;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoDouble leaveDistance;

   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;
   /** The current trajectory frame chosen by the user. */
   private ReferenceFrame currentTrajectoryFrame;

   /** The tangential plane is the frame in which the trajectory can be expressed in 2D. It is tangential to the final direction vector. */
   private final ReferenceFrame tangentialPlane;
   private final FrameQuaternion rotationPlane = new FrameQuaternion();
   private final AxisAngle axisAngleToWorld = new AxisAngle();

   // For viz
   private final boolean visualize;
   private final YoGraphicsList yoGraphicsList;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   private final YoBoolean showViz;

   public InitialClearancePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public InitialClearancePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public InitialClearancePositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public InitialClearancePositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry, boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.allowMultipleFrames = allowMultipleFrames;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      leaveTime = new YoDouble(namePrefix + "LeaveTime", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentTime = new YoDouble(namePrefix + "Time", registry);
      xyPolynomial = new YoPolynomial(namePrefix + "PositionPolynomial", 4, registry);
      zPolynomial = new YoPolynomial(namePrefix + "VelocityPolynomial", 7, registry);

      tangentialPlane = new ReferenceFrame("tangentialPlane", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            rotationPlane.changeFrame(getParent());
            transformToParent.setRotation(rotationPlane);
         }
      };

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame, tangentialPlane);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame, tangentialPlane);
      initialDirection = new YoFrameVectorInMultipleFrames(namePrefix + "InitialDirection", registry, referenceFrame, tangentialPlane);
      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame, tangentialPlane);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame, tangentialPlane);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame, tangentialPlane);

      leaveDistance = new YoDouble(namePrefix + "LeaveDistance", registry);

      multipleFramesHolders = new ArrayList<YoMultipleFramesHolder>();
      registerMultipleFramesHolders(initialPosition, finalPosition, currentPosition, currentVelocity, currentAcceleration);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
         final YoGraphicVector initialDirectionViz = new YoGraphicVector(namePrefix + "InitialDirection",
               initialPosition.buildUpdatedYoFramePointForVisualizationOnly(), initialDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.BlueViolet());
         yoGraphicsList = new YoGraphicsList(namePrefix + "FinalApproachTraj");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(initialDirectionViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               initialDirectionViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
               if (!visible)
                  bagOfBalls.hideAll();
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         yoGraphicsList = null;
         bagOfBalls = null;
         showViz = null;
      }
   }

   private void registerMultipleFramesHolders(YoMultipleFramesHolder... multipleFramesHolders)
   {
      for (YoMultipleFramesHolder multipleFramesHolder : multipleFramesHolders)
         this.multipleFramesHolders.add(multipleFramesHolder);
   }

   public void registerAndSwitchFrame(ReferenceFrame desiredFrame)
   {
      registerNewTrajectoryFrame(desiredFrame);
      switchTrajectoryFrame(desiredFrame);
   }

   public void registerNewTrajectoryFrame(ReferenceFrame newReferenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).registerReferenceFrame(newReferenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      changeFrame(referenceFrame, true);
   }

   private void changeFrame(ReferenceFrame referenceFrame, boolean checkIfAllowed)
   {
      if (checkIfAllowed)
         checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).switchCurrentReferenceFrame(referenceFrame);
   }

   public void setInitialPosition(double x, double y, double z)
   {
      this.initialPosition.set(x, y, z);
   }

   public void setInitialPosition(FramePoint3DReadOnly initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   public void setFinalPosition(double x, double y, double z)
   {
      this.finalPosition.set(x, y, z);
   }

   public void setFinalPosition(FramePoint3DReadOnly finalPosition)
   {
      this.finalPosition.set(finalPosition);
   }

   private final Vector3D tempVector = new Vector3D();

   public void setInitialClearance(FrameVector3DReadOnly initialDirection, double leaveDistance)
   {
      this.initialDirection.set(initialDirection);
      this.initialDirection.normalize();
      tempVector.set(this.initialDirection);
      EuclidGeometryTools.axisAngleFromZUpToVector3D(tempVector, axisAngleToWorld);
      rotationPlane.setIncludingFrame(this.initialDirection.getReferenceFrame(), axisAngleToWorld);

      this.leaveDistance.set(leaveDistance);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      leaveTime.set(defaultLeaveTimeInPercentOfTrajectoryTime * newTrajectoryTime);
   }

   public void setTrajectoryTime(double newTrajectoryTime, double leaveTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      MathTools.checkIntervalContains(leaveTime, 0.0, newTrajectoryTime);
      this.leaveTime.set(leaveTime);
   }

   public void initialize()
   {
      tangentialPlane.update();
      currentTrajectoryFrame = initialPosition.getReferenceFrame();

      changeFrame(tangentialPlane, false);

      currentTime.set(0.0);
      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      double tIntermediate = leaveTime.getDoubleValue();
      xyPolynomial.setCubic(tIntermediate, trajectoryTime.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);
      double z0 = initialPosition.getZ();
      double zIntermediate = initialPosition.getZ() + leaveDistance.getDoubleValue();
      double zf = finalPosition.getZ();
      zPolynomial.setSexticUsingWaypoint(0.0, tIntermediate, trajectoryTime.getDoubleValue(), z0, 0.0, 0.0, zIntermediate, zf, 0.0, 0.0);

      currentPosition.set(initialPosition);
      currentVelocity.setToZero();
      currentAcceleration.setToZero();

      changeFrame(currentTrajectoryFrame, false);

      if (visualize)
         visualizeTrajectory();
   }

   public void compute(double time)
   {
      tangentialPlane.update();
      changeFrame(tangentialPlane, false);

      this.currentTime.set(time);

      double tIntermediate = leaveTime.getDoubleValue();
      xyPolynomial.compute(MathTools.clamp(time, tIntermediate, trajectoryTime.getDoubleValue()));
      boolean shouldBeZero = currentTime.getDoubleValue() >= tIntermediate || currentTime.getDoubleValue() < 0.0;
      double alphaDot = shouldBeZero ? 0.0 : xyPolynomial.getVelocity();
      double alphaDDot = shouldBeZero ? 0.0 : xyPolynomial.getAcceleration();

      currentPosition.interpolate(initialPosition, finalPosition, xyPolynomial.getPosition());
      currentVelocity.sub(finalPosition, initialPosition);
      currentVelocity.scale(alphaDot);
      currentAcceleration.sub(finalPosition, initialPosition);
      currentAcceleration.scale(alphaDDot);

      zPolynomial.compute(MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue()));
      shouldBeZero = isDone() || currentTime.getDoubleValue() < 0.0;
      alphaDot = shouldBeZero ? 0.0 : zPolynomial.getVelocity();
      alphaDDot = shouldBeZero ? 0.0 : zPolynomial.getAcceleration();

      currentPosition.setZ(zPolynomial.getPosition());
      currentVelocity.setZ(zPolynomial.getVelocity());
      currentAcceleration.setZ(zPolynomial.getAcceleration());

      changeFrame(currentTrajectoryFrame, false);
   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = (double) i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         ballPosition.setIncludingFrame(currentPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(ballPosition);
      }
   }

   public void showVisualization()
   {
      if (!visualize)
         return;

      showViz.set(true);
   }

   public void hideVisualization()
   {
      if (!visualize)
         return;

      showViz.set(false);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(currentPosition);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAcceleration);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   private void checkIfMultipleFramesAllowed()
   {
      if (!allowMultipleFrames)
         throw new RuntimeException("Must set allowMultipleFrames to true in the constructor if you ever want to register a new frame.");
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public String toString()
   {
      String ret = "";

      ReferenceFrame currentFrame = initialPosition.getReferenceFrame();

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent position: " + currentPosition.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent velocity: " + currentVelocity.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent acceleration: " + currentAcceleration.toStringForASingleReferenceFrame(currentFrame);
      return ret;
   }
}
