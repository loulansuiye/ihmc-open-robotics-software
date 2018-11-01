package us.ihmc.avatar.footstepPlanning;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class MultiStageFootstepPlanningModule
{
   private static final boolean DEBUG = false;
   private static final double YO_VARIABLE_SERVER_DT = 0.01;
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 1;

   private final String name = getClass().getSimpleName();
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final String robotName;
   private final FullHumanoidRobotModel fullRobotModel;

   private final RealtimeRos2Node realtimeRos2Node;

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> taskScheduled = null;
   private ScheduledFuture<?> yoVariableServerScheduled = null;
   private final int updatePeriodMilliseconds = 1;

   private final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   private final YoVariableServer yoVariableServer;

   private final MultiStageFootstepPlanningController footstepPlanningController;

   public MultiStageFootstepPlanningModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                           DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.robotName = drcRobotModel.getSimpleRobotName();

      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.fullRobotModel = drcRobotModel.createFullRobotModel();
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, name));
      CommandInputManager commandInputManager = new CommandInputManager(name, FootstepPlannerCommunicationProperties.getSupportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(FootstepPlannerCommunicationProperties.getSupportedStatusMessages());
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(
            FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName), commandInputManager,
            FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName), statusOutputManager, realtimeRos2Node);

      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));

      footstepPlanningController = new MultiStageFootstepPlanningController(drcRobotModel.getContactPointParameters(),
                                                                            drcRobotModel.getFootstepPlannerParameters(), commandInputManager,
                                                                            statusOutputManager, executorService, registry, yoGraphicsListRegistry,
                                                                            Conversions.millisecondsToSeconds(DEFAULT_UPDATE_PERIOD_MILLISECONDS));

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, ToolboxStateMessage.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> receivedPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningRequestPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlannerParametersPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processPlannerParameters(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanningStatisticsRequestMessage.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> footstepPlanningController.processPlanningStatisticsRequest());
      IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher = ROS2Tools
            .createPublisher(realtimeRos2Node, TextToSpeechPacket.class, ROS2Tools::generateDefaultTopicName);

      footstepPlanningController.setTextToSpeechPublisher(textToSpeechPublisher);

      realtimeRos2Node.spin();

      yoVariableServer = startYoVariableServer();
   }

   private YoVariableServer startYoVariableServer()
   {
      if (!startYoVariableServer)
         return null;

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.TOOLBOX, YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      new Thread(() -> yoVariableServer.start()).start();

      yoVariableServerScheduled = executorService
            .scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);

      return yoVariableServer;
   }

   private Runnable createYoVariableServerRunnable(final YoVariableServer yoVariableServer)
   {
      return new Runnable()
      {
         double serverTime = 0.0;

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            serverTime += Conversions.millisecondsToSeconds(updatePeriodMilliseconds);
            yoVariableServer.update(Conversions.secondsToNanoseconds(serverTime));
         }
      };
   }

   public void receivedPacket(ToolboxStateMessage message)
   {
      if (taskScheduled != null)
      {
         return;
      }

      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
      {
      case WAKE_UP:
         footstepPlanningController.wakeUp();
         break;
      case REINITIALIZE:
         footstepPlanningController.reinitialize();
         break;
      case SLEEP:
         footstepPlanningController.sleep();
         break;
      }
   }

   public void destroy()
   {
      footstepPlanningController.sleep();

      if (yoVariableServerScheduled != null)
      {
         yoVariableServerScheduled.cancel(true);
      }
      executorService.shutdownNow();

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
      }
      realtimeRos2Node.destroy();

      if (DEBUG)
         PrintTools.debug(this, "Destroyed");
   }
}
