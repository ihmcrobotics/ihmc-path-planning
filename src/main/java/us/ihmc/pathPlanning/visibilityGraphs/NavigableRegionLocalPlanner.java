package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ClusterManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.LinearRegression3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegionLocalPlanner
{
   private static final boolean debug = false;

   private final List<Cluster> clusters = new ArrayList<>();
   private final List<PlanarRegion> regions = new ArrayList<>();
   private List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
   private final List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
   private final List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
   private ReferenceFrame localReferenceFrame;

   private double extrusionDistance = 0.80;
   private PlanarRegion homeRegion;

   private VisibilityMap localVisibilityMap;
   private ArrayList<Connection> connections = new ArrayList<>();


   private Point3D startLocationInLocalFrame;
   private Point3D goalLocationInLocalFrame;

   private ClusterManager clusterMgr;

   public NavigableRegionLocalPlanner(List<PlanarRegion> regions, PlanarRegion homeRegion,
                                      Point3D start, Point3D goal, double extrusionDistance)
   {
      this.regions.addAll(regions);

      this.homeRegion = homeRegion;

      this.extrusionDistance = extrusionDistance;

      createLocalReferenceFrame();

      FramePoint3D startFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), start);
      startFpt.changeFrame(localReferenceFrame);

      FramePoint3D goalFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), goal);
      goalFpt.changeFrame(localReferenceFrame);

      this.startLocationInLocalFrame = startFpt.getPoint();
      this.goalLocationInLocalFrame = goalFpt.getPoint();

      FramePoint3D test = new FramePoint3D(localReferenceFrame, goalLocationInLocalFrame);
      test.changeFrame(ReferenceFrame.getWorldFrame());
      //
      //      this.regions.remove(0);
   }

   private void createLocalReferenceFrame()
   {
      localReferenceFrame = new ReferenceFrame("regionLocalFrame", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = -2608124148208106613L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            homeRegion.getTransformToWorld(transformToParent);
         }
      };

      localReferenceFrame.update();
   }

   public ReferenceFrame getLocalReferenceFrame()
   {
      return localReferenceFrame;
   }

   public void processRegion()
   {
      regionsInsideHomeRegion = PlanarRegionTools.determineWhichRegionsAreInside(homeRegion, regions);

      if (debug)
      {
         System.out.println(regionsInsideHomeRegion.size());
      }

      classifyExtrusions(homeRegion);
      regionsInsideHomeRegion = filterRegionsThatAreBelow(regionsInsideHomeRegion, homeRegion);
      createClustersFromRegions(homeRegion, regionsInsideHomeRegion);
      createClusterForHomeRegion();

      clusterMgr = new ClusterManager();
      for (Cluster cluster : clusters)
      {
         clusterMgr.addCluster(cluster);
      }

      if (debug)
      {
         System.out.println("Extruding obstacles...");
      }

      // TODO The use of Double.MAX_VALUE for the observer seems rather risky. I'm actually surprised that it works.
      clusterMgr.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), extrusionDistance);

      for (Cluster cluster : clusters)
      {
         addExtraPointsInsideCluster(cluster);
      }

      //      addExtraPointsInsideCluster(clusters.get(0));

      VisibilityGraph localVisibilityGraph = new VisibilityGraph(clusterMgr);

      Point2D localStart = null;
      Point2D localGoal = null;

      ConvexPolygon2D homeConvexPol = homeRegion.getConvexHull();

      //check is any of the points are inside the homeRegion

      Point2D projectedStart2D = new Point2D(startLocationInLocalFrame.getX(), startLocationInLocalFrame.getY());
      Point2D projectedGoal2D = new Point2D(goalLocationInLocalFrame.getX(), goalLocationInLocalFrame.getY());

      if (homeConvexPol.isPointInside(projectedStart2D))
      {
         localStart = projectedStart2D;
         //         System.out.println(" ------------------------- Adding start position");
      }

      if (homeConvexPol.isPointInside(projectedGoal2D))
      {
         //         System.out.println("-------------------------- Adding goal position");
         localGoal = projectedGoal2D;
         //         javaFXMultiColorMeshBuilder.addSphere(0.05f, new Point3D(localGoal.getX(), localGoal.getY(), 0), Color.RED);
      }

      localVisibilityGraph.createStaticVisibilityMap(localStart, localGoal);
      localVisibilityMap = localVisibilityGraph.getVisibilityMap();
   }

   private List<PlanarRegion> filterRegionsThatAreBelow(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      for (PlanarRegion region : regionsToCheck)
      {
         if (!PlanarRegionTools.areAllPointsBelowTheRegion(region, homeRegion))
         {
            filteredList.add(region);
         }
      }

      return filteredList;
   }


   public void addExtraPointsInsideCluster(Cluster cluster)
   {
      if (debug)
      {
         System.out.println("Adding extra points");
      }
      PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal(), 0.25);
      if (debug)
      {
         System.out.println("Finished Adding extra points");
      }
   }

   public VisibilityMap getLocalVisibilityGraph()
   {
      return localVisibilityMap;
   }

   private void classifyExtrusions(PlanarRegion homeRegion)
   {
      for (PlanarRegion region : regionsInsideHomeRegion)
      {
         classifyExtrusion(region, homeRegion);
      }
   }

   private void createClusterForHomeRegion()
   {
      Cluster cluster = new Cluster();
      clusters.add(cluster);
      cluster.setType(Type.POLYGON);
      cluster.setTransformToWorld(localReferenceFrame.getTransformToWorldFrame());

      Point2D[] concaveHull = homeRegion.getConcaveHull();
      for (Point2D vertex : concaveHull)
      {
         cluster.addRawPointInLocal(vertex);
         //         javaFXMultiColorMeshBuilder.addSphere(0.05f, new Point3D(pointToProject.getX(), pointToProject.getY(), pointToProject.getZ()), Color.GREEN);
      }

      cluster.setClusterClosure(true);
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);
      cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
   }

   private void createClustersFromRegions(PlanarRegion homeRegion, List<PlanarRegion> regions)
   {
      for (PlanarRegion region : lineObstacleRegions)
      {
         if (regions.contains(region))
         {
            //            System.out.println("Creating a line cluster");
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.LINE);
            cluster.setTransformToWorld(localReferenceFrame.getTransformToWorldFrame());

            if (isRegionTooHighToStep(region, homeRegion))
            {
               cluster.setAdditionalExtrusionDistance(0);
            }
            else
            {
               //               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance * 0.6));

            }

            Vector3D normal = PlanarRegionTools.calculateNormal(homeRegion);
            ArrayList<Point3D> points = new ArrayList<>();
            for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
            {
               Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
               Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
               FramePoint3D fpt = new FramePoint3D();
               fpt.set(point3D);
               RigidBodyTransform transToWorld = new RigidBodyTransform();
               region.getTransformToWorld(transToWorld);
               fpt.applyTransform(transToWorld);

               Point3D pointToProject = fpt.getPoint();
               Point3D projectedPoint = new Point3D();
               EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);
               points.add(projectedPoint);
            }

            LinearRegression3D linearRegression = new LinearRegression3D(points);
            linearRegression.calculateRegression();

            //Convert to local frame
            Point3D[] extremes = linearRegression.getTheTwoPointsFurthestApart();
            FramePoint3D extreme1Fpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), extremes[0]);
            FramePoint3D extreme2Fpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), extremes[1]);

            cluster.addRawPointInWorld(extreme1Fpt.getPoint());
            cluster.addRawPointInWorld(extreme2Fpt.getPoint());

            //                           javaFXMultiColorMeshBuilder.addLine(extreme1Fpt.getPoint(), extreme2Fpt.getPoint(), 0.005, Color.BLUE);
         }
      }

      for (PlanarRegion region : polygonObstacleRegions)
      {
         if (regions.contains(region))
         {
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.POLYGON);
            cluster.setTransformToWorld(localReferenceFrame.getTransformToWorldFrame());

            Vector3D normal1 = PlanarRegionTools.calculateNormal(region);
            if (Math.abs(normal1.getZ()) >= 0.5)
            {
               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
            }

            Vector3D normal = PlanarRegionTools.calculateNormal(homeRegion);
            for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
            {
               Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
               Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
               FramePoint3D fpt = new FramePoint3D();
               fpt.set(point3D);
               RigidBodyTransform transToWorld = new RigidBodyTransform();
               region.getTransformToWorld(transToWorld);
               fpt.applyTransform(transToWorld);

               Point3D pointToProject = fpt.getPoint();
               Point3D projectedPoint = new Point3D();
               EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);

               FramePoint3D pointFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), projectedPoint);

               cluster.addRawPointInWorld(pointFpt.getPoint());
            }

            cluster.setClusterClosure(true);
         }
      }

      if (debug)
      {
         for (Cluster cluster : clusters)
         {
            System.out.println("Created a cluster of type: " + cluster.getType() + " with " + cluster.getRawPointsInLocal().size() + " points");
         }
      }
   }

   private void classifyExtrusion(PlanarRegion regionToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = PlanarRegionTools.calculateNormal(regionToProject);

      if (normal != null && regionToProject != regionToProjectTo)
      {
         if (Math.abs(normal.getZ()) < VisibilityGraphsParameters.NORMAL_Z_THRESHOLD_FOR_POLYGON_OBSTACLES)
         {
            lineObstacleRegions.add(regionToProject);
         }
         else
         {
            polygonObstacleRegions.add(regionToProject);
         }
      }

      //            System.out.println("Total obstacles to classify: " + regionsInsideHomeRegion.size() + "  Line obstacles: " + lineObstacleRegions.size()
      //                  + "   Polygon obstacles: " + polygonObstacleRegions.size());

   }

   private boolean isRegionTooHighToStep(PlanarRegion regionToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = PlanarRegionTools.calculateNormal(regionToProjectTo);

      for (int i = 0; i < regionToProject.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToProject.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToProject.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPoint = new Point3D();
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);

         if (pointToProject.distance(projectedPoint) >= VisibilityGraphsParameters.TOO_HIGH_TO_STEP_DISTANCE)
         {
            return true;
         }
      }

      return false;
   }

   public Point3D projectPointToPlane(Point3D pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = PlanarRegionTools.calculateNormal(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);

      Point3D projectedPoint = new Point3D();
      if (!EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint))
      {
         projectedPoint = null;
      }
      return projectedPoint;
   }


   public int getRegionId()
   {
      return homeRegion.getRegionId();
   }

   public List<Cluster> getClusters()
   {
      return clusters;
   }
}