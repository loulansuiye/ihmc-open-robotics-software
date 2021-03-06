cmake_minimum_required(VERSION 3.5)

project(controller_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
    "msg/AbortWalkingMessage.msg"
    "msg/AdjustFootstepMessage.msg"
    "msg/ArmDesiredAccelerationsMessage.msg"
    "msg/ArmTrajectoryMessage.msg"
    "msg/AtlasAuxiliaryRobotData.msg"
    "msg/AtlasDesiredPumpPSIPacket.msg"
    "msg/AtlasElectricMotorAutoEnableFlagPacket.msg"
    "msg/AtlasElectricMotorEnablePacket.msg"
    "msg/AtlasLowLevelControlModeMessage.msg"
    "msg/AtlasWristSensorCalibrationRequestPacket.msg"
    "msg/AutomaticManipulationAbortMessage.msg"
    "msg/BDIBehaviorCommandPacket.msg"
    "msg/BDIBehaviorStatusPacket.msg"
    "msg/BehaviorControlModePacket.msg"
    "msg/BehaviorControlModeResponsePacket.msg"
    "msg/BehaviorStatusPacket.msg"
    "msg/BlackFlyParameterPacket.msg"
    "msg/BoundingBox3DMessage.msg"
    "msg/BoundingBoxesPacket.msg"
    "msg/CapturabilityBasedStatus.msg"
    "msg/CenterOfMassTrajectoryMessage.msg"
    "msg/ChestHybridJointspaceTaskspaceTrajectoryMessage.msg"
    "msg/ChestTrajectoryMessage.msg"
    "msg/ClearDelayQueueMessage.msg"
    "msg/ComPositionPacket.msg"
    "msg/ComVelocityPacket.msg"
    "msg/ControllerCrashNotificationPacket.msg"
    "msg/DesiredAccelerationsMessage.msg"
    "msg/DetectedFacesPacket.msg"
    "msg/DetectedObjectPacket.msg"
    "msg/DoorLocationPacket.msg"
    "msg/DrillDetectionPacket.msg"
    "msg/EuclideanTrajectoryMessage.msg"
    "msg/EuclideanTrajectoryPointMessage.msg"
    "msg/ExoskeletonBehaviorStatePacket.msg"
    "msg/FisheyePacket.msg"
    "msg/FootLoadBearingMessage.msg"
    "msg/FootstepDataListMessage.msg"
    "msg/FootstepDataMessage.msg"
    "msg/FootstepPathPlanPacket.msg"
    "msg/FootstepPlanningRequestPacket.msg"
    "msg/FootstepPlanningToolboxOutputStatus.msg"
    "msg/FootstepPlanRequestPacket.msg"
    "msg/FootstepStatusMessage.msg"
    "msg/FootTrajectoryMessage.msg"
    "msg/FrameInformation.msg"
    "msg/GoHomeMessage.msg"
    "msg/HandCollisionDetectedPacket.msg"
    "msg/HandDesiredConfigurationMessage.msg"
    "msg/HandHybridJointspaceTaskspaceTrajectoryMessage.msg"
    "msg/HandJointAnglePacket.msg"
    "msg/HandLoadBearingMessage.msg"
    "msg/HandPowerCyclePacket.msg"
    "msg/HandTrajectoryMessage.msg"
    "msg/HeadHybridJointspaceTaskspaceTrajectoryMessage.msg"
    "msg/HeadTrajectoryMessage.msg"
    "msg/HeatMapPacket.msg"
    "msg/HeightQuadTreeLeafMessage.msg"
    "msg/HeightQuadTreeMessage.msg"
    "msg/HeightQuadTreeToolboxRequestMessage.msg"
    "msg/HighLevelStateChangeStatusMessage.msg"
    "msg/HighLevelStateMessage.msg"
    "msg/HumanoidBehaviorTypePacket.msg"
    "msg/HumanoidKinematicsToolboxConfigurationMessage.msg"
    "msg/IMUPacket.msg"
    "msg/IntrinsicParametersMessage.msg"
    "msg/InvalidPacketNotificationPacket.msg"
    "msg/JointspaceTrajectoryMessage.msg"
    "msg/KinematicsToolboxCenterOfMassMessage.msg"
    "msg/KinematicsToolboxConfigurationMessage.msg"
    "msg/KinematicsToolboxOutputStatus.msg"
    "msg/KinematicsToolboxRigidBodyMessage.msg"
    "msg/LegCompliancePacket.msg"
    "msg/LidarScanMessage.msg"
    "msg/LidarScanParametersMessage.msg"
    "msg/LoadBearingMessage.msg"
    "msg/LocalizationPacket.msg"
    "msg/LocalizationPointMapPacket.msg"
    "msg/LocalizationStatusPacket.msg"
    "msg/ManipulationAbortedStatus.msg"
    "msg/ManualHandControlPacket.msg"
    "msg/MessageCollection.msg"
    "msg/MessageCollectionNotification.msg"
    "msg/MomentumTrajectoryMessage.msg"
    "msg/MultisenseParameterPacket.msg"
    "msg/NeckDesiredAccelerationsMessage.msg"
    "msg/NeckTrajectoryMessage.msg"
    "msg/ObjectDetectorResultPacket.msg"
    "msg/ObjectWeightPacket.msg"
    "msg/OneDoFJointTrajectoryMessage.msg"
    "msg/PauseWalkingMessage.msg"
    "msg/PelvisHeightTrajectoryMessage.msg"
    "msg/PelvisOrientationTrajectoryMessage.msg"
    "msg/PelvisPoseErrorPacket.msg"
    "msg/PelvisTrajectoryMessage.msg"
    "msg/PilotAlarmPacket.msg"
    "msg/PilotInterfaceActionPacket.msg"
    "msg/PilotInterfacePacket.msg"
    "msg/PlanarRegionMessage.msg"
    "msg/PlanarRegionsListMessage.msg"
    "msg/PlanarVelocityPacket.msg"
    "msg/PlanOffsetStatus.msg"
    "msg/PointCloudWorldPacket.msg"
    "msg/Polygon2DMessage.msg"
    "msg/PrepareForLocomotionMessage.msg"
    "msg/QuadrupedBodyHeightMessage.msg"
    "msg/QuadrupedBodyOrientationMessage.msg"
    "msg/QuadrupedBodyPathPlanMessage.msg"
    "msg/QuadrupedControllerStateChangeMessage.msg"
    "msg/QuadrupedFootstepStatusMessage.msg"
    "msg/QuadrupedGroundPlaneMessage.msg"
    "msg/QuadrupedRequestedControllerStateMessage.msg"
    "msg/QuadrupedRequestedSteppingStateMessage.msg"
    "msg/QuadrupedStepMessage.msg"
    "msg/QuadrupedSteppingStateChangeMessage.msg"
    "msg/QuadrupedTimedStepListMessage.msg"
    "msg/QuadrupedTimedStepMessage.msg"
    "msg/QueueableMessage.msg"
    "msg/ReachingManifoldMessage.msg"
    "msg/RequestLidarScanMessage.msg"
    "msg/RequestPlanarRegionsListMessage.msg"
    "msg/RequestStereoPointCloudMessage.msg"
    "msg/RequestWristForceSensorCalibrationPacket.msg"
    "msg/RigidBodyExplorationConfigurationMessage.msg"
    "msg/RobotConfigurationData.msg"
    "msg/SCSListenerPacket.msg"
    "msg/SE3TrajectoryMessage.msg"
    "msg/SE3TrajectoryPointMessage.msg"
    "msg/SelectionMatrix3DMessage.msg"
    "msg/SimpleCoactiveBehaviorDataPacket.msg"
    "msg/SimulatedLidarScanPacket.msg"
    "msg/SnapFootstepPacket.msg"
    "msg/SO3TrajectoryMessage.msg"
    "msg/SO3TrajectoryPointMessage.msg"
    "msg/SoleTrajectoryMessage.msg"
    "msg/SpatialVectorMessage.msg"
    "msg/SpineDesiredAccelerationsMessage.msg"
    "msg/SpineTrajectoryMessage.msg"
    "msg/StampedPosePacket.msg"
    "msg/StateEstimatorModePacket.msg"
    "msg/StereoVisionPointCloudMessage.msg"
    "msg/StopAllTrajectoryMessage.msg"
    "msg/TextToSpeechPacket.msg"
    "msg/TimeIntervalMessage.msg"
    "msg/ToolboxStateMessage.msg"
    "msg/TrajectoryPoint1DMessage.msg"
    "msg/UIConnectedPacket.msg"
    "msg/UIPositionCheckerPacket.msg"
    "msg/ValveLocationPacket.msg"
    "msg/VehiclePosePacket.msg"
    "msg/VideoPacket.msg"
    "msg/WalkingControllerFailureStatusMessage.msg"
    "msg/WalkingStatusMessage.msg"
    "msg/WalkOverTerrainGoalPacket.msg"
    "msg/WalkToGoalBehaviorPacket.msg"
    "msg/WallPosePacket.msg"
    "msg/WaypointBasedTrajectoryMessage.msg"
    "msg/WeightMatrix3DMessage.msg"
    "msg/WholeBodyTrajectoryMessage.msg"
    "msg/WholeBodyTrajectoryToolboxConfigurationMessage.msg"
    "msg/WholeBodyTrajectoryToolboxMessage.msg"
    "msg/WholeBodyTrajectoryToolboxOutputStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
               geometry_msgs
               trajectory_msgs
               builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()