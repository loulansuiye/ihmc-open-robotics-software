package us.ihmc.quadrupedRobotics.controller;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedControllerStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedWalkingControllerState;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.output.JointIntegratorComponent;
import us.ihmc.quadrupedRobotics.output.OutputProcessorBuilder;
import us.ihmc.quadrupedRobotics.output.StateChangeSmootherComponent;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal
 * finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedControllerManager implements RobotController, CloseableAndDisposable
{
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> requestedControllerState = new YoEnum<>("requestedControllerState", registry, HighLevelControllerName.class,
                                                                                         true);
   private final AtomicReference<HighLevelControllerName> requestedControllerStateReference = new AtomicReference<>();
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;
   private final OutputProcessor outputProcessor;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final HighLevelStateChangeStatusMessage stateChangeMessage = new HighLevelStateChangeStatusMessage();
   private final WalkingControllerFailureStatusMessage walkingControllerFailureStatusMessage = new WalkingControllerFailureStatusMessage();

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties, HighLevelControllerName initialControllerState)
   {
      this.controllerToolbox = new QuadrupedControllerToolbox(runtimeEnvironment, physicalProperties, registry, runtimeEnvironment.getGraphicsListRegistry());
      this.runtimeEnvironment = runtimeEnvironment;

      // Initialize control modules
      this.controlManagerFactory = new QuadrupedControlManagerFactory(controllerToolbox, physicalProperties, runtimeEnvironment.getGraphicsListRegistry(),
                                                                      registry);

      commandInputManager = new CommandInputManager(QuadrupedControllerAPIDefinition.getQuadrupedSupportedCommands());
      try
      {
         commandInputManager.registerConversionHelper(new ClearDelayQueueConverter(QuadrupedControllerAPIDefinition.getQuadrupedSupportedCommands()));
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         e.printStackTrace();
      }
      statusMessageOutputManager = new StatusMessageOutputManager(QuadrupedControllerAPIDefinition.getQuadrupedSupportedStatusMessages());

      controlManagerFactory.getOrCreateFeetManager();
      controlManagerFactory.getOrCreateBodyOrientationManager();
      controlManagerFactory.getOrCreateBalanceManager();
      controlManagerFactory.getOrCreateJointSpaceManager();

      // Initialize output processor
      StateChangeSmootherComponent stateChangeSmootherComponent = new StateChangeSmootherComponent(runtimeEnvironment, registry);
      JointIntegratorComponent jointControlComponent = new JointIntegratorComponent(runtimeEnvironment, registry);
      controlManagerFactory.getOrCreateFeetManager().attachStateChangedListener(stateChangeSmootherComponent.createFiniteStateMachineStateChangedListener());
      OutputProcessorBuilder outputProcessorBuilder = new OutputProcessorBuilder(runtimeEnvironment.getFullRobotModel());
      outputProcessorBuilder.addComponent(stateChangeSmootherComponent);
      outputProcessorBuilder.addComponent(jointControlComponent);
      outputProcessor = outputProcessorBuilder.build();

      requestedControllerState.set(null);
      requestedControllerState.addVariableChangedListener(v ->
      {
         HighLevelControllerName currentRequestedState = requestedControllerState.getEnumValue();
         if (currentRequestedState != null)
         {
            requestedControllerStateReference.set(currentRequestedState);
            requestedControllerState.set(null);
         }
      });

      if(initialControllerState == null)
      {
         initialControllerState = runtimeEnvironment.getHighLevelControllerParameters().getDefaultInitialControllerState();
      }

      this.stateMachine = buildStateMachine(runtimeEnvironment, initialControllerState);
   }

   public State getState(HighLevelControllerName state)
   {
      return stateMachine.getState(state);
   }

   @Override
   public void initialize()
   {
      outputProcessor.initialize();
   }

   @Override
   public void doControl()
   {
      if (commandInputManager.isNewCommandAvailable(HighLevelControllerStateCommand.class))
      {
         requestedControllerState.set(commandInputManager.pollNewestCommand(HighLevelControllerStateCommand.class).getHighLevelControllerName());
      }

      // update controller state machine
      stateMachine.doActionAndTransition();

      // update contact state used for state estimation
      switch (stateMachine.getCurrentStateKey())
      {
      case DO_NOTHING_BEHAVIOR:
      case STAND_PREP_STATE:
      case STAND_READY:
      case STAND_TRANSITION_STATE:
      case FREEZE_STATE:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitch(false);
         }

         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_RIGHT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_RIGHT).setFootContactState(true);
         break;
      default:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitch(true);
            if (controllerToolbox.getContactState(robotQuadrant) == ContactState.IN_CONTACT)
            {
               runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(true);
            }
            else
            {
               runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(false);
            }
         }
         break;
      }

      // update fall detector
      if (controllerToolbox.getFallDetector().detect())
      {
         walkingControllerFailureStatusMessage.falling_direction_.set(runtimeEnvironment.getFullRobotModel().getRootJoint().getLinearVelocityForReading());
         statusMessageOutputManager.reportStatusMessage(walkingControllerFailureStatusMessage);
      }

      // update output processor
      outputProcessor.update();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return "A proxy controller for switching between multiple subcontrollers";
   }

   public RobotMotionStatusHolder getMotionStatusHolder()
   {
      return motionStatusHolder;
   }

   private StateMachine<HighLevelControllerName, HighLevelControllerState> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment, HighLevelControllerName initialControllerState)
   {
      OneDoFJoint[] controlledJoints = runtimeEnvironment.getFullRobotModel().getControllableOneDoFJoints();
      HighLevelControllerParameters highLevelControllerParameters = runtimeEnvironment.getHighLevelControllerParameters();
      JointDesiredOutputList jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

      DoNothingControllerState doNothingState = new DoNothingControllerState(controlledJoints, highLevelControllerParameters);
      StandPrepControllerState standPrepState = new StandPrepControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      StandReadyControllerState standReadyState = new StandReadyControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      QuadrupedWalkingControllerState walkingState = new QuadrupedWalkingControllerState(runtimeEnvironment, controllerToolbox, commandInputManager,
                                                                                         statusMessageOutputManager, controlManagerFactory);
      FreezeControllerState freezeState = new FreezeControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      SmoothTransitionControllerState standTransitionState = new SmoothTransitionControllerState("toWalking", HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                                                 standReadyState, walkingState, controlledJoints,
                                                                                                 highLevelControllerParameters);
      SmoothTransitionControllerState exitWalkingState = new SmoothTransitionControllerState("exitWalking", HighLevelControllerName.EXIT_WALKING, walkingState,
                                                                                             freezeState, controlledJoints, highLevelControllerParameters);

      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("controller").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());

      factory.addState(HighLevelControllerName.DO_NOTHING_BEHAVIOR, doNothingState);
      factory.addState(HighLevelControllerName.STAND_PREP_STATE, standPrepState);
      factory.addState(HighLevelControllerName.STAND_READY, standReadyState);
      factory.addState(HighLevelControllerName.FREEZE_STATE, freezeState);
      factory.addState(HighLevelControllerName.WALKING, walkingState);
      factory.addState(HighLevelControllerName.STAND_TRANSITION_STATE, standTransitionState);
      factory.addState(HighLevelControllerName.EXIT_WALKING, exitWalkingState);

      // Always allow all transitions to DO_NOTHING and FREEZE
      for (HighLevelControllerName state : HighLevelControllerName.values)
      {
         if(!state.equals(HighLevelControllerName.DO_NOTHING_BEHAVIOR))
         {
            factory.addTransition(state, HighLevelControllerName.DO_NOTHING_BEHAVIOR, createRequestedTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR));
         }
         else if(!state.equals(HighLevelControllerName.FREEZE_STATE))
         {
            factory.addTransition(state, HighLevelControllerName.FREEZE_STATE, createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
         }
      }

      // Set up standard operating transitions
      factory.addTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.STAND_PREP_STATE, createRequestedTransition(HighLevelControllerName.STAND_PREP_STATE));
      factory.addTransition(HighLevelControllerName.FREEZE_STATE, HighLevelControllerName.STAND_PREP_STATE, createRequestedTransition(HighLevelControllerName.STAND_PREP_STATE));
      factory.addDoneTransition(HighLevelControllerName.STAND_PREP_STATE, HighLevelControllerName.STAND_READY);
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.STAND_TRANSITION_STATE, createRequestedTransition(HighLevelControllerName.STAND_TRANSITION_STATE));
      factory.addDoneTransition(HighLevelControllerName.STAND_TRANSITION_STATE, HighLevelControllerName.WALKING);

      // Set up walking controller failure transition
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      BooleanProvider isFallDetected = controllerToolbox.getFallDetector().getIsFallDetected();
      factory.addTransition(HighLevelControllerName.WALKING, fallbackControllerState, t -> isFallDetected.getValue());

      factory.addStateChangedListener((to, from) ->
      {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            stateChangeMessage.setInitialHighLevelControllerName(fromByte);
            stateChangeMessage.setEndHighLevelControllerName(toByte);
            statusMessageOutputManager.reportStatusMessage(stateChangeMessage);
      });

      registry.addChild(doNothingState.getYoVariableRegistry());
      registry.addChild(standPrepState.getYoVariableRegistry());
      registry.addChild(standReadyState.getYoVariableRegistry());
      registry.addChild(freezeState.getYoVariableRegistry());
      registry.addChild(walkingState.getYoVariableRegistry());
      registry.addChild(standTransitionState.getYoVariableRegistry());
      registry.addChild(exitWalkingState.getYoVariableRegistry());

      return factory.build(initialControllerState);
   }

   private StateTransitionCondition createRequestedTransition(HighLevelControllerName endState)
   {
      return time -> requestedControllerState.getEnumValue() == endState;
   }

   public void createControllerNetworkSubscriber(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(subscriberTopicNameGenerator, commandInputManager,
                                                                                                publisherTopicNameGenerator, statusMessageOutputManager,
                                                                                                realtimeRos2Node);
      controllerNetworkSubscriber.addMessageCollector(QuadrupedControllerAPIDefinition.createDefaultMessageIDExtractor());
      controllerNetworkSubscriber.addMessageValidator(QuadrupedControllerAPIDefinition.createDefaultMessageValidation());
   }

   public void resetSteppingState()
   {
      QuadrupedWalkingControllerState steppingState = (QuadrupedWalkingControllerState) stateMachine.getState(HighLevelControllerName.WALKING);
      steppingState.onEntry();
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }
}
