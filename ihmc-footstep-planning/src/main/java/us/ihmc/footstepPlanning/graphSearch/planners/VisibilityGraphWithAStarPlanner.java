package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class VisibilityGraphWithAStarPlanner implements FootstepPlanner
{
   private static final boolean DEBUG = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private static final double defaultTimeout = 5.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timeout = new YoDouble("timeout", registry);

   private final YoBoolean hasPath = new YoBoolean("hasPath", registry);
   private final YoDouble timeSpentBeforeFootstepPlanner = new YoDouble("timeSpentBeforeFootstepPlanner", registry);
   private final YoDouble timeSpentInFootstepPlanner = new YoDouble("timeSpentInFootstepPlanner", registry);
   private final YoDouble planningHorizonLength = new YoDouble("planningHorizonLength", registry);
   private final YoEnum<FootstepPlanningResult> yoResult = new YoEnum<>("planningResult", registry, FootstepPlanningResult.class);
   private final NavigableRegionsManager navigableRegionsManager;

   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlanner bodyPathPlanner;
   private final BodyPathHeuristics heuristics;
   private final FootstepPlanner footstepPlanner;

   private PlanarRegionsList planarRegionsList;
   private final FramePose3D bodyStartPose = new FramePose3D();
   private final FramePose3D bodyGoalPose = new FramePose3D();
   private final List<Point3DReadOnly> waypoints = new ArrayList<>();

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 100;
   private final List<YoFramePoint3D> bodyPathPoints = new ArrayList<>();

   private final ListOfStatistics listOfStatistics = new ListOfStatistics();
   private final VisibilityGraphStatistics visibilityGraphStatistics = new VisibilityGraphStatistics();

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons,
                                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this("", parameters, footPolygons, graphicsListRegistry, parentRegistry);
   }

   public VisibilityGraphWithAStarPlanner(String prefix, FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons,
                                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
      heuristics = new BodyPathHeuristics(parameters.getCostParameters().getVisGraphWithAStarHeuristicsWeight(), parameters, bodyPathPlanner);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons, parameters);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludePitchAndRollCost(false);
      costBuilder.setIncludeHeightCost(false);

      FootstepCost footstepCost = costBuilder.buildCost();

      planningHorizonLength.set(1.0);

      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, registry);

      this.navigableRegionsManager = new NavigableRegionsManager(new YoVisibilityGraphParameters(new DefaultVisibilityGraphParameters(), registry));

      timeout.set(defaultTimeout);
      visualizing = graphicsListRegistry != null;
      if (visualizing)
      {
         setupVisualization(prefix, graphicsListRegistry, registry);
      }
   }

   private void setupVisualization(String prefix, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(prefix + "VisGraph");

      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         YoFramePoint3D point = new YoFramePoint3D(prefix + "BodyPathPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         point.setToNaN();
         bodyPathPoints.add(point);
         YoGraphicPosition pointVisualization = new YoGraphicPosition(prefix + "BodyPathPoint" + i, point, 0.02, YoAppearance.Yellow());
         yoGraphicsList.add(pointVisualization);
      }

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (DEBUG)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      bodyStartPose.setToZero(stanceFrame);
      bodyStartPose.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);

      hasPath.set(false);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      bodyGoalPose.setIncludingFrame(goal.getGoalPoseBetweenFeet());

      hasPath.set(false);
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      footstepPlanner.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public double getPlanningDuration()
   {
      return timeSpentBeforeFootstepPlanner.getDoubleValue() + timeSpentInFootstepPlanner.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
      planningHorizonLength.set(planningHorizon);

      hasPath.set(false);
   }

   @Override
   public FootstepPlanningResult planPath()
   {
      long startTime = System.currentTimeMillis();
      waypoints.clear();

      if (planarRegionsList == null)
      {
         waypoints.add(new Point3D(bodyStartPose.getPosition()));
         waypoints.add(new Point3D(bodyGoalPose.getPosition()));
      }
      else
      {
         Point3DReadOnly startPos = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
         Point3DReadOnly goalPos = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);
         navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

         if (startPos == null)
         {
            PrintTools.info("adding plane at start foot");
            startPos = new Point3D(bodyStartPose.getX(), bodyStartPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyStartPose.getX(), bodyStartPose.getY());
         }
         if (goalPos == null)
         {
            PrintTools.info("adding plane at goal pose");
            goalPos = new Point3D(bodyGoalPose.getX(), bodyGoalPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyGoalPose.getX(), bodyGoalPose.getY());
         }

         if (DEBUG)
         {
            PrintTools.info("Starting to plan using " + getClass().getSimpleName());
            PrintTools.info("Body start pose: " + startPos);
            PrintTools.info("Body goal pose:  " + goalPos);
         }

         try
         {
            List<Point3DReadOnly> path = new ArrayList<>(navigableRegionsManager.calculateBodyPath(startPos, goalPos));

            if (path.size() < 2)
            {
               if (parameters.getReturnBestEffortPlan())
               {
                  Vector2D goalDirection = new Vector2D(bodyGoalPose.getPosition());
                  goalDirection.sub(bodyStartPose.getX(), bodyStartPose.getY());
                  goalDirection.scale(planningHorizonLength.getDoubleValue() / goalDirection.length());
                  Point3D waypoint = new Point3D(bodyStartPose.getPosition());
                  waypoint.add(goalDirection.getX(), goalDirection.getY(), 0.0);
                  waypoints.add(waypoint);
               }
               else
               {
                  double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
                  timeSpentBeforeFootstepPlanner.set(seconds);
                  timeSpentInFootstepPlanner.set(0.0);
                  yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
                  return yoResult.getEnumValue();
               }
            }

            for (Point3DReadOnly waypoint3d : path)
            {
               waypoints.add(new Point3D(waypoint3d));
            }
         }
         catch (Exception e)
         {
            e.printStackTrace();
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforeFootstepPlanner.set(seconds);
            timeSpentInFootstepPlanner.set(0.0);
            yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
         }
      }

      bodyPathPlanner.setWaypoints(waypoints);
      bodyPathPlanner.compute();

      if (visualizing)
      {
         updateBodyPathVisualization();
      }
      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPathPlanner.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPathPlanner.getPointAlongPath(alpha, goalPose2d);
      heuristics.setGoalAlpha(alpha);

      FramePose3D footstepPlannerGoal = new FramePose3D();
      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      footstepPlannerGoal.setOrientationYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(footstepPlannerGoal);
      footstepPlanner.setGoal(goal);

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforeFootstepPlanner.set(seconds);

      hasPath.set(true);

      yoResult.set(FootstepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      if (!hasPath.getBooleanValue())
      {
         FootstepPlanningResult pathResult = planPath();
         if (!pathResult.validForExecution())
            return pathResult;
      }

      footstepPlanner.setTimeout(timeout.getDoubleValue() - timeSpentBeforeFootstepPlanner.getDoubleValue());

      long startTime = System.currentTimeMillis();
      yoResult.set(footstepPlanner.plan());
      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInFootstepPlanner.set(seconds);

      if (DEBUG)
      {
         PrintTools.info("Visibility graph with A* planner finished. Result: " + yoResult.getEnumValue());
      }

      return yoResult.getEnumValue();
   }

   // TODO hack to add start and goal planar regions
   private void addPlanarRegionAtZeroHeight(double xLocation, double yLocation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, 0.0)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   private void updateBodyPathVisualization()
   {
      Pose2D tempPose = new Pose2D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPathPlanner.getPointAlongPath(percent, tempPose);
         Point3D position = new Point3D();
         position.set(tempPose.getPosition());
         Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList);
         if (projectedPoint != null)
         {
            bodyPathPoints.get(i).set(projectedPoint);
         }
         else
         {
            bodyPathPoints.get(i).setToNaN();
         }
      }
   }

   public Point3DReadOnly[][] getNavigableRegions()
   {
      return navigableRegionsManager.getNavigableExtrusions();
   }

   public List<Point3DReadOnly> getBodyPathWaypoints()
   {
      return waypoints;
   }

   public Pose2D getLowLevelPlannerGoal()
   {
      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPathPlanner.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPathPlanner.getPointAlongPath(alpha, goalPose2d);
      return goalPose2d;
   }

   public BodyPathPlanner getBodyPathPlanner()
   {
      return bodyPathPlanner;
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

   @Override
   public ListOfStatistics getPlannerStatistics()
   {
      listOfStatistics.clear();

      packVisibilityGraphStatistics(visibilityGraphStatistics);

      listOfStatistics.addStatistics(visibilityGraphStatistics);

      return listOfStatistics;
   }

   private void packVisibilityGraphStatistics(VisibilityGraphStatistics statistics)
   {
      VisibilityMapHolder startMap = navigableRegionsManager.getStartMap();
      VisibilityMapHolder goalMap = navigableRegionsManager.getGoalMap();
      VisibilityMapHolder interRegionsMap = navigableRegionsManager.getInterRegionConnections();
      List<NavigableRegion> navigableRegions = navigableRegionsManager.getNavigableRegions();

      statistics.setStartVisibilityMapInWorld(startMap.getMapId(), startMap.getVisibilityMapInWorld());
      statistics.setGoalVisibilityMapInWorld(goalMap.getMapId(), goalMap.getVisibilityMapInWorld());
      statistics.setInterRegionsVisibilityMapInWorld(interRegionsMap.getMapId(), interRegionsMap.getVisibilityMapInWorld());
      statistics.addNavigableRegions(navigableRegions);
   }
}
