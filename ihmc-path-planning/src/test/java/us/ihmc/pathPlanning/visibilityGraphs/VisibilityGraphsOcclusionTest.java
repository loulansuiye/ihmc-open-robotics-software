package us.ihmc.pathPlanning.visibilityGraphs;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class VisibilityGraphsOcclusionTest
{
   private static final DefaultVisibilityGraphParameters VISIBILITY_GRAPH_PARAMETERS = new DefaultVisibilityGraphParameters()
   {
      @Override
      public int getPlanarRegionMinSize()
      {
         return 0;
      }
   };

   private static final boolean VERBOSE = false;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int rays = 5000;
   private static final int maxPolygonsToVisualize = 10;
   private static final int maxPolygonsVertices = 50;
   private static final double defaultMaxAllowedSolveTime = 1.0;
   private static final int bodyPathVisualizationResolution = 500;
   private static final double marchingSpeedInMetersPerTick = 0.50;
   private static final int maxNumberOfIterations = 40;

   @Rule
   public TestName name = new TestName();

   @Test(timeout = 300000)
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testFlatGround()
   {
      Point3D startPose = new Point3D();
      Point3D goalPose = new Point3D();
      PlanarRegionsList regions = createFlatGround(startPose, goalPose);
      runTest(startPose, goalPose, regions, true, defaultMaxAllowedSolveTime);
   }

   @Test(timeout = 300000)
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testSimpleOcclusions()
   {
      Point3D startPose = new Point3D();
      Point3D goalPose = new Point3D();
      PlanarRegionsList regions = createSimpleOcclusionField(startPose, goalPose);
      runTest(startPose, goalPose, regions, true, defaultMaxAllowedSolveTime);
   }

   @Test(timeout = 300000)
   @Ignore
   public void testMazeWithOcclusions()
   {
      Point3D startPose = new Point3D();
      Point3D goalPose = new Point3D();
      PlanarRegionsList regions = createMazeOcclusionField(startPose, goalPose);
      runTest(startPose, goalPose, regions, true, defaultMaxAllowedSolveTime);
   }

   private void runTest(Point3D start, Point3D goal, PlanarRegionsList regions, boolean cheatByAddingGroundRegion, double maxAllowedSolveTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      NavigableRegionsManager vizGraphs = new NavigableRegionsManager(VISIBILITY_GRAPH_PARAMETERS);

      SimulationConstructionSet scs = null;

      YoFramePoint currentPosition = new YoFramePoint("CurrentPosition", worldFrame, registry);
      currentPosition.set(start);

      YoFramePoint observerPoint = null;
      List<YoFramePoint> rayIntersectionVisualizations = null;
      List<YoFrameConvexPolygon2d> visiblePolygons = null;
      List<YoFramePose> visiblePolygonPoses = null;
      List<YoGraphicPolygon> polygonVisualizations = null;

      BagOfBalls bodyPathViz = null;
      BagOfBalls bodyPathWaypointsViz = null;

      YoBoolean plannerFailed = new YoBoolean("PlannerFailed", registry);
      YoDouble solveTime = new YoDouble("SolveTime", registry);

      if (visualize)
      {
         YoFramePoint yoStart = new YoFramePoint("start", worldFrame, registry);
         yoStart.set(start);
         YoFramePoint yoGoal = new YoFramePoint("goal", worldFrame, registry);
         yoGoal.set(start);

         visiblePolygons = new ArrayList<>();
         visiblePolygonPoses = new ArrayList<>();
         polygonVisualizations = new ArrayList<>();
         for (int i = 0; i < maxPolygonsToVisualize; i++)
         {
            YoFrameConvexPolygon2d polygon = new YoFrameConvexPolygon2d("Polygon" + i, worldFrame, maxPolygonsVertices, registry);
            YoFramePose pose = new YoFramePose("PolygonPose" + i, worldFrame, registry);
            pose.setToNaN();
            visiblePolygons.add(polygon);
            visiblePolygonPoses.add(pose);
            YoGraphicPolygon visualization = new YoGraphicPolygon("Polygon" + i, polygon, pose.getPosition(), pose.getOrientation(), 1.0, 0.02,
                                                                  new YoAppearanceRGBColor(Color.BLUE, 0.8));
            polygonVisualizations.add(visualization);
            graphicsListRegistry.registerYoGraphic("viz", visualization);
            graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(visualization);
         }

         rayIntersectionVisualizations = new ArrayList<>();
         for (int i = 0; i < rays; i++)
         {
            YoFramePoint point = new YoFramePoint("RayIntersection" + i, ReferenceFrame.getWorldFrame(), registry);
            point.setToNaN();
            YoGraphicPosition visualization = new YoGraphicPosition("RayIntersection" + i, point, 0.0025, YoAppearance.Blue());
            rayIntersectionVisualizations.add(point);
            graphicsListRegistry.registerYoGraphic("viz", visualization);
         }

         observerPoint = new YoFramePoint("Observer", worldFrame, registry);
         observerPoint.setToNaN();
         YoGraphicPosition observerVisualization = new YoGraphicPosition("Observer", observerPoint, 0.05, YoAppearance.Red());
         graphicsListRegistry.registerYoGraphic("viz", observerVisualization);

         YoGraphicPosition currentPositionVisualization = new YoGraphicPosition("CurrentPosition", currentPosition, 0.05, YoAppearance.Blue());
         graphicsListRegistry.registerYoGraphic("viz", currentPositionVisualization);

         bodyPathViz = new BagOfBalls(bodyPathVisualizationResolution, 0.01, "bodyPath", registry, graphicsListRegistry);
         bodyPathWaypointsViz = new BagOfBalls(100, 0.025, YoAppearance.Yellow(), registry, graphicsListRegistry);

         scs = setupSCS(name.getMethodName(), registry, regions, start, goal);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setInPoint();
      }

      double maxSolveTime = 0.0;

      // Add the ground plane here so the visibility graph works. Remove that later.
      PlanarRegionsList visiblePlanarRegions;
      if (cheatByAddingGroundRegion)
         visiblePlanarRegions = new PlanarRegionsList(regions.getPlanarRegion(0));
      else
         visiblePlanarRegions = new PlanarRegionsList();

      int iteration = -1;

      while (!currentPosition.getFrameTuple().epsilonEquals(goal, 1.0e-3))
      {
         iteration++;

         if (VERBOSE)
            PrintTools.info("iteration: " + iteration);

         if (iteration > maxNumberOfIterations)
         {
            PrintTools.info("Too many iterations too reach goal.");
            break;
         }
         Point3D observer = new Point3D(currentPosition.getFrameTuple());
         observer.addZ(0.05);

         visiblePlanarRegions = createVisibleRegions(regions, observer, visiblePlanarRegions, rayIntersectionVisualizations);

         if (visualize)
         {
            observerPoint.set(observer);

            for (int polygonIdx = 0; polygonIdx < maxPolygonsToVisualize; polygonIdx++)
            {
               visiblePolygonPoses.get(polygonIdx).setToNaN();
            }
            int polygons = Math.min(maxPolygonsToVisualize, visiblePlanarRegions.getNumberOfPlanarRegions());
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            FramePose pose = new FramePose();
            for (int polygonIdx = 0; polygonIdx < polygons; polygonIdx++)
            {
               PlanarRegion planarRegion = visiblePlanarRegions.getPlanarRegion(polygonIdx);
               if (planarRegion.getConvexHull().getNumberOfVertices() > visiblePolygons.get(polygonIdx).getMaxNumberOfVertices())
               {
                  throw new RuntimeException("Increase max number of vertices for visualization.");
               }
               planarRegion.getTransformToWorld(transformToWorld);
               pose.setPose(transformToWorld);
               visiblePolygonPoses.get(polygonIdx).set(pose);
               visiblePolygons.get(polygonIdx).setConvexPolygon2d(planarRegion.getConvexHull());
            }
         }

         vizGraphs.setPlanarRegions(visiblePlanarRegions.getPlanarRegionsAsList());

         List<Point3D> bodyPath = null;

         try
         {
            long startTime = System.currentTimeMillis();
            bodyPath = vizGraphs.calculateBodyPath(currentPosition.getPoint3dCopy(), goal);
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            solveTime.set(seconds);

            if (seconds > maxSolveTime)
            {
               maxSolveTime = seconds;
            }

            if (bodyPath == null)
            {
               PrintTools.info("Planner failed: body path is null.");
               plannerFailed.set(true);
            }
         }
         catch (Exception e)
         {
            // The catch needs to be removed once the visibility graph is improved.
            PrintTools.info("Planner threw exception:");
            e.printStackTrace();
            plannerFailed.set(true);
         }

         if (bodyPath == null || !bodyPath.get(bodyPath.size() - 1).geometricallyEquals(goal, 1.0e-3))
         {
            if (visualize)
            {
               scs.setTime(iteration);
               scs.tickAndUpdate();
            }
            PrintTools.info("Failed");
            plannerFailed.set(true);
         }

         if (currentPosition.getPoint3dCopy().distance(goal) < 0.05)
         {
            if (bodyPath.size() > 2)
            {
               PrintTools.info("Start is next to goal, should be a straight line but had: " + bodyPath.size() + " waypoints.");
               plannerFailed.set(true);
            }
         }

         if (plannerFailed.getBooleanValue())
            break;

         if (visualize)
         {
            visualizeBodyPath(bodyPath, bodyPathViz);
            bodyPathWaypointsViz.hideAll();
            bodyPath.forEach(bodyPathWaypointsViz::setBall);
            scs.setTime(iteration);
            scs.tickAndUpdate();
         }

         currentPosition.set(travelAlongBodyPath(marchingSpeedInMetersPerTick, currentPosition.getPoint3dCopy(), bodyPath));
      }

      PrintTools.info("Maximum solve time was " + maxSolveTime + "s.");

      if (visualize)
      {
         scs.setOutPoint();
         scs.cropBuffer();
         scs.setPlaybackRealTimeRate(0.001);
         scs.play();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertTrue("Planner took too long: " + maxSolveTime + "s.", maxSolveTime < maxAllowedSolveTime);
         Assert.assertFalse("Planner failed at least once.", plannerFailed.getBooleanValue());
      }
   }

   private static void visualizeBodyPath(List<Point3D> bodyPath, BagOfBalls vizToUpdate)
   {
      int numberOfBalls = vizToUpdate.getNumberOfBalls();

      double bodyPathLength = 0.0;
      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         bodyPathLength += bodyPath.get(i).distance(bodyPath.get(i + 1));
      }

      double distanceToTravel = bodyPathLength / (numberOfBalls - 1.0);
      Point3D position = new Point3D(bodyPath.get(0));
      vizToUpdate.setBall(position);

      for (int i = 0; i < numberOfBalls - 1; i++)
      {
         position = travelAlongBodyPath(distanceToTravel, position, bodyPath);
         vizToUpdate.setBall(position, i);
      }
   }

   private static Point3D travelAlongBodyPath(double distanceToTravel, Point3D startingPosition, List<Point3D> bodyPath)
   {
      Point3D newPosition = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i), bodyPath.get(i + 1));

         if (segment.distance(startingPosition) < 1.0e-10)
         {
            Vector3D segmentDirection = segment.getDirection(true);
            newPosition.scaleAdd(distanceToTravel, segmentDirection, startingPosition);

            if (segment.distance(newPosition) < 1.0e-10)
            {
               return newPosition;
            }
            else
            {
               distanceToTravel -= startingPosition.distance(segment.getSecondEndpoint());
               startingPosition = new Point3D(segment.getSecondEndpoint());
            }
         }
      }

      return new Point3D(startingPosition);
   }

   private static SimulationConstructionSet setupSCS(String testName, YoVariableRegistry testRegistry, PlanarRegionsList regions, Point3D start, Point3D goal)
   {
      Robot robot = new Robot(VisibilityGraphsOcclusionTest.class.getSimpleName());
      robot.addYoVariableRegistry(testRegistry);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.8);
      if (regions != null)
      {
         graphics3DObject.addPlanarRegionsList(regions, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
         scs.setGroundVisible(false);
      }

      graphics3DObject.identity();
      graphics3DObject.translate(start);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Green());

      graphics3DObject.identity();
      graphics3DObject.translate(goal);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Red());

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraPosition(-7.0, -1.0, 25.0);
      scs.setCameraFix(0.0, 0.0, 0.0);

      return scs;
   }

   private PlanarRegionsList createVisibleRegions(PlanarRegionsList regions, Point3D observer, PlanarRegionsList knownRegions,
                                                  List<YoFramePoint> rayPointsToPack)
   {
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(observer, 1.0, rays);
      List<ConvexPolygon2D> visiblePolygons = new ArrayList<>();
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         visiblePolygons.add(new ConvexPolygon2D());
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      for (int rayIndex = 0; rayIndex < rays; rayIndex++)
      {
         Point3D pointOnSphere = pointsOnSphere[rayIndex];
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, observer);
         Point3D intersection = PlanarRegionTools.intersectRegionsWithRay(regions, observer, rayDirection);
         if (intersection == null)
         {
            if (rayPointsToPack != null)
            {
               rayPointsToPack.get(rayIndex).setToNaN();
            }
            continue;
         }

         if (rayPointsToPack != null)
         {
            rayPointsToPack.get(rayIndex).set(intersection);
         }
         for (int regionIdx = 0; regionIdx < regions.getNumberOfPlanarRegions(); regionIdx++)
         {
            PlanarRegion region = regions.getPlanarRegion(regionIdx);
            if (PlanarRegionTools.isPointOnRegion(region, intersection, 0.01))
            {
               region.getTransformToWorld(transform);
               Point3D pointOnPlane = new Point3D(intersection);
               pointOnPlane.applyInverseTransform(transform);

               Point2D newVertex = new Point2D();
               newVertex.set(pointOnPlane);

               visiblePolygons.get(regionIdx).addVertex(newVertex);
            }
         }
      }

      PlanarRegionsList visible = new PlanarRegionsList();
      for (int i = 0; i < visiblePolygons.size(); i++)
      {
         ConvexPolygon2D polygon = visiblePolygons.get(i);
         polygon.update();
         if (polygon.getNumberOfVertices() < 2)
         {
            continue;
         }

         PlanarRegion originalRegion = regions.getPlanarRegion(i);
         originalRegion.getTransformToWorld(transform);
         PlanarRegion newRegion = new PlanarRegion(transform, polygon);
         visible.addPlanarRegion(newRegion);
      }

      return combine(knownRegions, visible);
   }

   private PlanarRegionsList combine(PlanarRegionsList regionsA, PlanarRegionsList regionsB)
   {
      PlanarRegionsList ret = new PlanarRegionsList();

      boolean[] added = new boolean[regionsB.getNumberOfPlanarRegions()];
      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         added[regionBIdx] = false;
      }

      for (PlanarRegion regionA : regionsA.getPlanarRegionsAsList())
      {
         RigidBodyTransform transformA = new RigidBodyTransform();
         regionA.getTransformToWorld(transformA);
         boolean foundMatchingRegion = false;

         for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
         {
            PlanarRegion regionB = regionsB.getPlanarRegion(regionBIdx);
            RigidBodyTransform transformB = new RigidBodyTransform();
            regionB.getTransformToWorld(transformB);
            if (transformA.epsilonEquals(transformB, 0.01))
            {
               ConvexPolygon2D newHull = new ConvexPolygon2D(regionA.getConvexHull(), regionB.getConvexHull());
               ret.addPlanarRegion(new PlanarRegion(transformA, newHull));
               foundMatchingRegion = true;
               added[regionBIdx] = true;
            }
         }

         if (!foundMatchingRegion)
         {
            ret.addPlanarRegion(new PlanarRegion(transformA, new ConvexPolygon2D(regionA.getConvexHull())));
         }
      }

      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         if (!added[regionBIdx])
         {
            ret.addPlanarRegion(regionsB.getPlanarRegion(regionBIdx));
         }
      }

      return ret;
   }

   private PlanarRegionsList createFlatGround(Point3D startPoseToPack, Point3D goalPoseToPack)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addCubeReferencedAtBottomMiddle(50.0, 5.0, 0.005);

      startPoseToPack.set(-23.005, -2.001, 0.0025);
      //      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), startPoseToPack, startPoseToPack);

      goalPoseToPack.set(23.005, 2.001, 0.0025);
      //      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), goalPoseToPack, goalPoseToPack);

      return generator.getPlanarRegionsList();
   }

   private PlanarRegionsList createSimpleOcclusionField(Point3D startPoseToPack, Point3D goalPoseToPack)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      //      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.addRectangle(6.0, 6.0);
      generator.translate(-1.0, -1.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 4.0);
      generator.identity();
      //      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(1.0, 1.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 4.0);

      startPoseToPack.set(-2.005, -2.001, 0.0);
      //      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), startPoseToPack, startPoseToPack);

      goalPoseToPack.set(2.005, 2.001, 0.0);
      //      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), goalPoseToPack, goalPoseToPack);

      return generator.getPlanarRegionsList();
   }

   private PlanarRegionsList createMazeOcclusionField(Point3D startPoseToPack, Point3D goalPoseToPack)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.addRectangle(6.0, 12.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(-1.0, -2.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 8.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(1.0, 0.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 8.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(0.0, -4.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.X);
      generator.addRectangle(2.0, 1.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(0.0, 4.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.X);
      generator.addRectangle(2.0, 1.0);

      startPoseToPack.set(-2.0, -5.0, 0.0);
      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), startPoseToPack, startPoseToPack);

      goalPoseToPack.set(0.0, -5.0, 0.0);
      RotationMatrixTools.applyRollRotation(Math.toRadians(10.0), goalPoseToPack, goalPoseToPack);

      return generator.getPlanarRegionsList();
   }
}
