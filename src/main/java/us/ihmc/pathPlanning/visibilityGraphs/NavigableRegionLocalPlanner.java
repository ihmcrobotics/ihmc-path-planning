package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

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
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ClusterManager;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.tools.LinearRegression3D;
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

   double extrusionDistance = 0.80;
   PlanarRegion homeRegion;

   SimpleWeightedGraph<Point2D, DefaultWeightedEdge> localVisibilityMap;

   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   Point3D startLocationInLocalFrame;
   Point3D goalLocationInLocalFrame;

   ClusterManager clusterMgr;

   public NavigableRegionLocalPlanner(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder, List<PlanarRegion> regions, PlanarRegion homeRegion,
                                      Point3D start, Point3D goal, double extrusionDistance)
   {
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
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

   public boolean isPointInsideTheRegion(Point3D point)
   {
      FramePoint3D fpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), point.getX(), point.getY(), point.getZ());
      fpt.changeFrame(localReferenceFrame);

      return homeRegion.isPointInside(new Point2D(fpt.getX(), fpt.getY()));
   }

   public ReferenceFrame getLocalReferenceFrame()
   {
      return localReferenceFrame;
   }

   public void processRegion()
   {
      regionsInsideHomeRegion = determineWhichRegionsAreInside(homeRegion, regions);

      if (debug)
      {
         System.out.println(regionsInsideHomeRegion.size());
      }

      classifyExtrusions(homeRegion);
      regionsInsideHomeRegion = filterRegionsThatAreBelow(regionsInsideHomeRegion, homeRegion);
      createClustersFromRegions(homeRegion, regionsInsideHomeRegion);
      createClusterForHomeRegion();

      clusterMgr = new ClusterManager();
      clusterMgr.setVis(javaFXMultiColorMeshBuilder);
      for (Cluster cluster : clusters)
      {
         clusterMgr.addCluster(cluster);
      }

      if (debug)
      {
         System.out.println("Extruding obstacles...");
      }

      clusterMgr.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), extrusionDistance);

      //Visuals local frame
      //
      //      for (Cluster cluster : clusters)
      //      {
      //         for (int i = 0; i < cluster.getRawPointsInCluster().size(); i++)
      //         {
      //            javaFXMultiColorMeshBuilder.addSphere(0.03f,
      //                                                  new Point3D(cluster.getRawPointsInCluster().get(i).getX(), cluster.getRawPointsInCluster().get(i).getY(), 0),
      //                                                  Color.RED);
      //         }
      //
      //         for (int i = 0; i < cluster.getListOfNonNavigableExtrusions().size(); i++)
      //         {
      //            javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(cluster.getListOfNonNavigableExtrusions().get(i).getX(),
      //                                                                     cluster.getListOfNonNavigableExtrusions().get(i).getY(), 0),
      //                                                  Color.RED);
      //         }
      //
      //         for (int i = 1; i < cluster.getListOfNonNavigableExtrusions().size(); i++)
      //         {
      //            javaFXMultiColorMeshBuilder.addLine(new Point3D(cluster.getListOfNonNavigableExtrusions().get(i - 1).getX(),
      //                                                            cluster.getListOfNonNavigableExtrusions().get(i - 1).getY(), 0),
      //                                                new Point3D(cluster.getListOfNonNavigableExtrusions().get(i).getX(),
      //                                                            cluster.getListOfNonNavigableExtrusions().get(i).getY(), 0),
      //                                                0.005, Color.ORANGE);
      //         }
      //         for (int i = 1; i < cluster.getListOfNavigableExtrusions().size(); i++)
      //         {
      //            javaFXMultiColorMeshBuilder.addLine(new Point3D(cluster.getListOfNavigableExtrusions().get(i - 1).getX(),
      //                                                            cluster.getListOfNavigableExtrusions().get(i - 1).getY(), 0),
      //                                                new Point3D(cluster.getListOfNavigableExtrusions().get(i).getX(),
      //                                                            cluster.getListOfNavigableExtrusions().get(i).getY(), 0),
      //                                                0.005, Color.GREEN);
      //         }
      //      }

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
         if (!areAllPointsBelowTheRegion(region, homeRegion))
         {
            filteredList.add(region);
         }
      }

      return filteredList;
   }

   private boolean areAllPointsBelowTheRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      for (int i = 0; i < homeRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D homeRegionPoint = new FramePoint3D();
         homeRegionPoint.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         homeRegionPoint.applyTransform(transToWorld);

         for (int j = 0; j < regionToCheck.getConvexHull().getNumberOfVertices(); j++)
         {
            Point2D point2D1 = (Point2D) regionToCheck.getConvexHull().getVertex(j);
            Point3D point3D1 = new Point3D(point2D1.getX(), point2D1.getY(), 0);
            FramePoint3D otherRegionPoint = new FramePoint3D();
            otherRegionPoint.set(point3D1);
            RigidBodyTransform transToWorld1 = new RigidBodyTransform();
            regionToCheck.getTransformToWorld(transToWorld1);
            otherRegionPoint.applyTransform(transToWorld1);

            if (homeRegionPoint.getZ() + 0.1 < otherRegionPoint.getZ())
            {
               if (debug)
               {
                  System.out.println(homeRegionPoint.getZ() + "   " + otherRegionPoint.getZ());
               }

               return false;
            }
         }
      }

      return true;
   }

   public void addExtraPointsInsideCluster(Cluster cluster)
   {
      if (debug)
      {
         System.out.println("Adding extra points");
      }
      PointCloudTools.doBrakeDownOn2DPoints(cluster.getListOfNavigableExtrusions(), 0.25);
      if (debug)
      {
         System.out.println("Finished Adding extra points");
      }
   }

   public SimpleWeightedGraph<Point2D, DefaultWeightedEdge> getLocalVisibilityGraph()
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

      for (int i = 0; i < homeRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D pointToProject = fpt.getPoint();

         FramePoint3D ptfpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), pointToProject);
         ptfpt.changeFrame(localReferenceFrame);

         cluster.addRawPoint(ptfpt.getPoint());
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

            if (isRegionTooHighToStep(region, homeRegion))
            {
               cluster.setAdditionalExtrusionDistance(0);
            }
            else
            {
               //               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance * 0.6));

            }

            Vector3D normal = calculateNormal(homeRegion);
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
            extreme1Fpt.changeFrame(localReferenceFrame);
            extreme2Fpt.changeFrame(localReferenceFrame);

            cluster.addRawPoint(extreme1Fpt.getPoint());
            cluster.addRawPoint(extreme2Fpt.getPoint());

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

            Vector3D normal1 = calculateNormal(region);
            if (Math.abs(normal1.getZ()) >= 0.5)
            {
               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
            }

            Vector3D normal = calculateNormal(homeRegion);
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
               pointFpt.changeFrame(localReferenceFrame);

               cluster.addRawPoint(pointFpt.getPoint());
            }

            cluster.setClusterClosure(true);
         }
      }

      if (debug)
      {
         for (Cluster cluster : clusters)
         {
            System.out.println("Created a cluster of type: " + cluster.getType() + " with " + cluster.getRawPointsInCluster().size() + " points");
         }
      }
   }

   private ArrayList<PlanarRegion> determineWhichRegionsAreInside(PlanarRegion containingRegion, List<PlanarRegion> otherRegionsEx)
   {
      ArrayList<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();

      for (PlanarRegion otherRegion : otherRegionsEx)
      {
         if (isPartOfTheRegionInside(otherRegion, containingRegion))
         {
            regionsInsideHomeRegion.add(otherRegion);
         }
      }

      return regionsInsideHomeRegion;
   }

   private boolean isPartOfTheRegionInside(PlanarRegion regionToCheck, PlanarRegion containingRegion)
   {
      ArrayList<Point3D> pointsToCalculateCentroid = new ArrayList<>();
      Point2D[] homePointsArr = new Point2D[containingRegion.getConvexHull().getNumberOfVertices()];
      for (int i = 0; i < containingRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) containingRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         containingRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D transformedPt = fpt.getPoint();

         homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
         pointsToCalculateCentroid.add(new Point3D(transformedPt.getX(), transformedPt.getY(), transformedPt.getZ()));
      }

      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

      Point3D centroidOfHomeRegion = PointCloudTools.getCentroid(pointsToCalculateCentroid);

      Vector3D normal = calculateNormal(containingRegion);

      for (int i = 0; i < regionToCheck.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToCheck.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToCheck.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         //         javaFXMultiColorMeshBuilder.addSphere(0.05f, new Point3D(fpt.getX(), fpt.getY(), fpt.getZ()), Color.YELLOW);

         //         if (fpt.getPoint().getZ() < centroidOfHomeRegion.getZ())
         //         {
         //            return false;
         //         }
         //
         //         if (fpt.getZ() > centroidOfHomeRegion.getZ() && fpt.getZ() < (centroidOfHomeRegion.getZ() + 1.5))
         //         {
         //            //            System.out.println("Region is higher that home region and atlas");
         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPointFromOtherRegion = new Point3D();

         //                     System.out.println(pointToProject + "  " + point3D + "  " + normal + "  " + projectedPointFromOtherRegion);
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPointFromOtherRegion);

         if (homeConvexPol.isPointInside(new Point2D(projectedPointFromOtherRegion.getX(), projectedPointFromOtherRegion.getY())))
         {
            return true;
         }
         //         }
      }

      return false;
   }

   private void classifyExtrusion(PlanarRegion regionToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal(regionToProject);

      if (normal != null && regionToProject != regionToProjectTo)
      {
         if (Math.abs(normal.getZ()) < 0.8)
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
      Vector3D normal = calculateNormal(regionToProjectTo);

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

         if (pointToProject.distance(projectedPoint) >= 0.5)
         {
            return true;
         }
      }

      return false;
   }

   public Point3D projectPointToPlane(Point3D pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
      //      FramePoint3D fpt = new FramePoint3D();
      //      fpt.set(point3D);
      //      RigidBodyTransform transToWorld = new RigidBodyTransform();
      //      regionToProjectTo.getTransformToWorld(transToWorld);
      //      fpt.applyTransform(transToWorld);

      Point3D projectedPoint = new Point3D();
      if (!EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint))
      {
         projectedPoint = null;
      }
      return projectedPoint;
   }

   //   private void classifyRegions(ArrayList<PlanarRegion> regions)
   //   {
   //      for (PlanarRegion region : regions)
   //      {
   //         Vector3D normal = calculateNormal(region);
   //
   //         if (normal != null)
   //         {
   //            if (Math.abs(normal.getZ()) <= 0.98) //0.98
   //            {
   //               obstacleRegions.add(region);
   //            }
   //            else
   //            {
   //               accesibleRegions.add(region);
   //            }
   //         }
   //      }
   //   }

   private Vector3D calculateNormal(PlanarRegion region)
   {
      if (!region.getConvexHull().isEmpty())
      {
         FramePoint3D fpt1 = new FramePoint3D();
         fpt1.set(new Point3D(region.getConvexHull().getVertex(0).getX(), region.getConvexHull().getVertex(0).getY(), 0));
         RigidBodyTransform trans = new RigidBodyTransform();
         region.getTransformToWorld(trans);
         fpt1.applyTransform(trans);

         FramePoint3D fpt2 = new FramePoint3D();
         fpt2.set(new Point3D(region.getConvexHull().getVertex(1).getX(), region.getConvexHull().getVertex(1).getY(), 0));
         fpt2.applyTransform(trans);

         FramePoint3D fpt3 = new FramePoint3D();
         fpt3.set(new Point3D(region.getConvexHull().getVertex(2).getX(), region.getConvexHull().getVertex(2).getY(), 0));
         fpt3.applyTransform(trans);

         Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(fpt1.getPoint(), fpt2.getPoint(), fpt3.getPoint());
         return normal;
      }
      return null;
   }

   public ArrayList<Point3D> loadPointCloudFromFile(String fileName)
   {
      ArrayList<Cluster> clusters = new ArrayList<>();
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         //br = new BufferedReader(new FileReader(FILENAME));
         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         double averageX = 0.0;
         double averageY = 0.0;
         double averageZ = 0.0;

         int index = 0;

         Cluster cluster = new Cluster();
         int nPacketsRead = 0;

         ArrayList<Point3D> pointsTemp = new ArrayList<>();

         while ((sCurrentLine = br.readLine()) != null)
         {
            //            System.out.println(sCurrentLine);

            if (sCurrentLine.contains("PR_"))
            {
               if (!pointsTemp.isEmpty())
               {
                  cluster.addRawPoints(pointsTemp, true);
                  pointsTemp.clear();
               }

               cluster = new Cluster();
               clusters.add(cluster);
               nPacketsRead++;
               //               System.out.println("New cluster created");
            }

            else if (sCurrentLine.contains("RBT,"))
            {
               //               System.out.println("Transformation read");
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());

               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);
               Vector3D translation = new Vector3D(x, y, z);

               float qx = (float) Double.parseDouble(points[3]);
               float qy = (float) Double.parseDouble(points[4]);
               float qz = (float) Double.parseDouble(points[5]);
               float qs = (float) Double.parseDouble(points[6]);
               Quaternion quat = new Quaternion(qx, qy, qz, qs);

               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(quat, translation);
               cluster.setTransform(rigidBodyTransform);
            }
            else
            {
               //               System.out.println("adding point");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);

               pointsTemp.add(new Point3D(x, y, z));

               averageX = averageX + x;
               averageY = averageY + y;
               averageZ = averageZ + z;

               index++;
            }
         }

         for (Cluster cluster1 : clusters)
         {
            ArrayList<Point2D> vertices = new ArrayList<>();

            for (Point3D pt : cluster1.getRawPointsInCluster())
            {
               vertices.add(new Point2D(pt.getX(), pt.getY()));
            }

            ConvexPolygon2D convexPolygon = new ConvexPolygon2D(vertices);

            PlanarRegion planarRegion = new PlanarRegion(cluster1.getTransform(), convexPolygon);

            regions.add(planarRegion);
         }

         System.out.println("Loaded " + regions.size() + " regions");
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }

      }
      return null;

   }
}