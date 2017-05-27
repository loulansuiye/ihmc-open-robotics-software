package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class PelvisHeightControlState extends PelvisAndCenterOfMassHeightControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /** We take the spatialFeedback command from the RigidBodyTaskspaceControlState and pack it into a point feedback command and set the selection matrix to Z only**/
   private final PointFeedbackControlCommand pointFeedbackCommand = new PointFeedbackControlCommand();
   private final SelectionMatrix6D linearZSelectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D linearZWeightMatrix = new WeightMatrix6D();
   
   /** When we handle the PelvisTrajectoryCommand we pull out the z component and pack it into another PelvisTrajectoryCommand**/
   private final PelvisTrajectoryCommand tempPelvisTrajectoryCommand = new PelvisTrajectoryCommand();

   /** handles the trajectory and the queuing**/
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   
   private final ReferenceFrame baseFrame;
   private final DoubleYoVariable defaultHeightAboveAnkleForHome;
   private final FramePose tempPose = new FramePose();
   
   public PelvisHeightControlState(YoPositionPIDGainsInterface gains, HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      super(PelvisHeightControlMode.USER);
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      RigidBody elevator = fullRobotModel.getElevator();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      baseFrame = referenceFrames.getMidFootZUpGroundFrame();
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      

      taskspaceControlState = new RigidBodyTaskspaceControlState("Height", pelvis, elevator, elevator, trajectoryFrames, pelvisFrame, baseFrame, yoTime, null, graphicsListRegistry, registry);
      taskspaceControlState.setGains(null, gains);
      
      // the nominalHeightAboveAnkle is from the ankle to the pelvis, we need to add the ankle to sole frame to get the proper home height
      double soleToAnkleZHeight = computeSoleToAnkleMeanZHeight(controllerToolbox, fullRobotModel);
      defaultHeightAboveAnkleForHome = new DoubleYoVariable(getClass().getSimpleName() + "DefaultHeightAboveAnkleForHome", registry);
      defaultHeightAboveAnkleForHome.set(walkingControllerParameters.nominalHeightAboveAnkle() + soleToAnkleZHeight);
      
      parentRegistry.addChild(registry);
   }
   
   /**
    * the nominalHeightAboveAnkle is from the ankle to the pelvis, we need to add the ankle to sole frame to get the proper home height
    * @param controllerToolbox
    * @param fullRobotModel
    * @return the z height between the ankle and the sole frame
    */
   private double computeSoleToAnkleMeanZHeight(HighLevelHumanoidControllerToolbox controllerToolbox, FullHumanoidRobotModel fullRobotModel)
   {
      double zHeight = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         zHeight += ankleToSole.getTranslationZ();
      }
      zHeight /= 2.0;
      return zHeight;
   }
   
   /**
    * set the qp weights for the taskspace linear z command
    * @param linearWeight
    */
   public void setWeights(Vector3D linearWeight)
   {
      taskspaceControlState.setWeights(null, linearWeight);
   }

   @Override
   public void doAction()
   {
      taskspaceControlState.doAction();
   }
   
   /**
    * check that the command is valid and queue the trajectory
    * @param command
    * @param initialPose the initial pelvis position
    * @return whether the command passed validation and was queued
    */
   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command, FramePose initialPose)
   {
      if (command.useCustomControlFrame())
      {
         PrintTools.warn("Can not use custom control frame with pelvis orientation.");
         return false;
      }
      
      // We have to remove the orientation and xy components of the command, and adjust the selection matrix;
      // We do this to break up the pelvis control, it reduces the complexity of each manager at the expense of these little hacks.
      tempPelvisTrajectoryCommand.set(command);
      
      //set the selection matrix to z only
      SelectionMatrix6D commandSelectionMatrix = tempPelvisTrajectoryCommand.getSelectionMatrix();
      if(commandSelectionMatrix != null)
      {
         linearZSelectionMatrix.set(commandSelectionMatrix);
         ReferenceFrame linearSelectionFrame = linearZSelectionMatrix.getLinearSelectionFrame();
         if(linearSelectionFrame != null && !linearSelectionFrame.isZupFrame())
         {
            PrintTools.warn("Selection Matrix Linear Frame was not Z up, PelvisTrajectoryCommand can only handle Selection matrix linear components with Z up frames.");
            return false;
         }
      }
      else
      {
         linearZSelectionMatrix.clearLinearSelection();
      }
      
      linearZSelectionMatrix.clearAngularSelection();
      linearZSelectionMatrix.setLinearAxisSelection(false, false, true);
      linearZSelectionMatrix.setSelectionFrame(ReferenceFrame.getWorldFrame());
      tempPelvisTrajectoryCommand.setSelectionMatrix(linearZSelectionMatrix);
      
      //set the weight matrix to z only
      WeightMatrix6D commanedWeightMatrix = tempPelvisTrajectoryCommand.getWeightMatrix();
      if(commanedWeightMatrix != null)
      {
         linearZWeightMatrix.set(commanedWeightMatrix);
         ReferenceFrame linearWeightFrame = linearZWeightMatrix.getLinearWeightFrame();
         if(linearWeightFrame != null && !linearWeightFrame.isZupFrame())
         {
            PrintTools.warn("Weight Matrix Linear Frame was not Z up, PelvisTrajectoryCommand can only handle weight matrix linear components with Z up frames.");
            return false;
         }
      }
      else
      {
         linearZWeightMatrix.clearLinearWeights();
      }
      linearZWeightMatrix.clearAngularWeights();
      WeightMatrix3D weightLinearPart = linearZWeightMatrix.getLinearPart();
      linearZWeightMatrix.setLinearWeights(0.0, 0.0, weightLinearPart.getZAxisWeight());
      linearZWeightMatrix.setWeightFrame(ReferenceFrame.getWorldFrame());
      tempPelvisTrajectoryCommand.setWeightMatrix(linearZWeightMatrix);
      
      return taskspaceControlState.handlePoseTrajectoryCommand(tempPelvisTrajectoryCommand, initialPose);
   }
   
   /**
    * returns the control frame which is fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
    * @return
    */
   public ReferenceFrame getControlFrame()
   {
      return taskspaceControlState.getControlFrame();
   }
   
   /**
    * Packs positionToPack with the current desired height, The parameter's frame will be set to the trajectory frame
    */
   @Override
   public void getCurrentDesiredHeight(FramePoint positionToPack)
   {
      taskspaceControlState.getDesiredPose(tempPose);
      tempPose.getPositionIncludingFrame(positionToPack);
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      taskspaceControlState.holdCurrent();
   }

   /**
    * sets the desired height to defaultHeightAboveAnkleForHome in baseFrame (MidFootZUpGroundFrame)
    */
   @Override
   public void goHome(double trajectoryTime)
   {
      tempPose.setToZero(baseFrame);
      tempPose.setZ(defaultHeightAboveAnkleForHome.getDoubleValue());
      taskspaceControlState.goToPoseFromCurrent(tempPose, trajectoryTime);
   }

   /**
    * If the command says to stop then set the desired to the actual
    */
   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if(command.isStopAllTrajectory())
      {
         initializeDesiredHeightToCurrent();
      }
   }

   /**
    * Returns 0.0, we don't compute the acceleration here, we send the command to a feedback controller to do it for us
    */
   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager)
   {
      //Consider using the previous desired linear momentum z to be more consistent with the CenterOfMassheightManager
      return 0.0;
   }
   
   
   private final FramePoint controlPosition = new FramePoint();
   private final FrameOrientation controlOrientation = new FrameOrientation();
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   
   /**
    * returns the point feedback command for the z height of the pelvis
    */
   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      //We have to do some nasty copying, because the taskspaceControlState returns a spatial feedback command, but the controller core doesn't like
      //when you send two overlapping commands (pelvis orientation uses orientation feedback comand)
      SpatialFeedbackControlCommand spatialFeedbackControlCommand = taskspaceControlState.getSpatialFeedbackControlCommand();

      SpatialAccelerationCommand spcatialAccelerationCommand = spatialFeedbackControlCommand.getSpatialAccelerationCommand();
      pointFeedbackCommand.getSpatialAccelerationCommand().set(spcatialAccelerationCommand);
      
      pointFeedbackCommand.set(spatialFeedbackControlCommand.getBase(), spatialFeedbackControlCommand.getEndEffector());
      spatialFeedbackControlCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      pointFeedbackCommand.set(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      pointFeedbackCommand.setControlBaseFrame(spatialFeedbackControlCommand.getControlBaseFrame());
      pointFeedbackCommand.setGains(spatialFeedbackControlCommand.getGains().getPositionGains());
      spatialFeedbackControlCommand.getControlFramePoseIncludingFrame(controlPosition, controlOrientation);
      pointFeedbackCommand.setBodyFixedPointToControl(controlPosition);
      
      return pointFeedbackCommand;
   }
}