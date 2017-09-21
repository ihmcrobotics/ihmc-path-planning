package newz;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.scene.paint.Color;
import newz.Cluster.ExtrusionSide;
import newz.Cluster.Type;
import tools.LinearRegression;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegionLocalPlanner
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   ArrayList<PlanarRegion> accesibleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> obstacleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
   ArrayList<PlanarRegion> lineObstacleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> polygonObstacleRegions = new ArrayList<>();

   double extrusionDistance = 0.60;
   Point2D startingPosition = new Point2D(1.5, 0);
   PlanarRegion homeRegion;

   SimpleWeightedGraph<Point2D, DefaultWeightedEdge> localVisibilityMap;

   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   Point3D start;
   Point3D goal;

   public NavigableRegionLocalPlanner(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder, ArrayList<PlanarRegion> regions, PlanarRegion homeRegion,
                                      Point3D start, Point3D goal)
   {
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
      this.regions = regions;
      this.homeRegion = homeRegion;
      this.start = start;
      this.goal = goal;
      regions.remove(0);
   }

   public void processRegion()
   {
      regionsInsideHomeRegion = determineWhichRegionsAreInside(homeRegion, regions);
      //      regionsInsideHomeRegion.add(regions.get(1));

      classifyExtrusions(homeRegion);
      createClustersFromRegions(homeRegion, regionsInsideHomeRegion);
      createClusterForHomeRegion();

      ClusterMgr clusterMgr = new ClusterMgr();
      for (Cluster cluster : clusters)
      {
         clusterMgr.addCluster(cluster);
      }

      System.out.println("Extruding obstacles...");

      clusterMgr.performExtrusions(startingPosition, extrusionDistance);

      //Visuals
      //      DebugSphere sph = new DebugSphere(basicJMEInterface, 0.25f, 30, 30, ColorRGBA.Red);
      //      basicJMEInterface.getZUpNode().attachChild(sph);
      //      sph.setLocalTranslation((float) startingPosition.getX32(), (float) startingPosition.getY32(), 0);

      for (Cluster cluster : clusters)
      {
         //         if(cluster.getType() == Type.LINE)
         //         {
         for (int i = 1; i < cluster.getListOfNonNavigableExtrusions().size(); i++)
         {
            javaFXMultiColorMeshBuilder.addLine(new Point3D(cluster.getListOfNonNavigableExtrusions().get(i - 1).getX(),
                                                            cluster.getListOfNonNavigableExtrusions().get(i - 1).getY(), 0),
                                                new Point3D(cluster.getListOfNonNavigableExtrusions().get(i).getX(),
                                                            cluster.getListOfNonNavigableExtrusions().get(i).getY(), 0),
                                                0.005, Color.ORANGE);
         }
         for (int i = 1; i < cluster.getListOfNavigableExtrusions().size(); i++)
         {
            javaFXMultiColorMeshBuilder.addLine(new Point3D(cluster.getListOfNavigableExtrusions().get(i - 1).getX(),
                                                            cluster.getListOfNavigableExtrusions().get(i - 1).getY(), 0),
                                                new Point3D(cluster.getListOfNavigableExtrusions().get(i).getX(),
                                                            cluster.getListOfNavigableExtrusions().get(i).getY(), 0),
                                                0.005, Color.GREEN);
         }
         //         }
      }

      //      for (PlanarRegion region : lineObstacleRegions)
      //      {
      //         visualizeRegion(region, ColorRGBA.Red);
      //      }
      //
      //      for (PlanarRegion region : polygonObstacleRegions)
      //      {
      //         visualizeRegion(region, ColorRGBA.Orange);
      //      }

      VisibilityGraph localVisibilityGraph = new VisibilityGraph(clusterMgr);

      Point2D localStart = null;
      Point2D localGoal = null;

      //Convert HomeRegion into a convex2d that is updated with its transform
      Point2D[] homePointsArr = new Point2D[homeRegion.getConvexHull().getNumberOfVertices()];
      for (int i = 0; i < homeRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D transformedPt = fpt.getPoint();

         homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
      }
      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

      //check is any of the points are inside the homeRegion
      Point3D projectedStart = projectPointToPlane(start, homeRegion);
      Point3D projectedGoal = projectPointToPlane(goal, homeRegion);

      Point2D projectedStart2D = new Point2D(projectedStart.getX(), projectedStart.getY());
      Point2D projectedGoal2D = new Point2D(projectedGoal.getX(), projectedGoal.getY());

      if (homeConvexPol.isPointInside(projectedStart2D))
      {
         localStart = projectedStart2D;
         System.out.println(" ------------------------- Adding start position");
      }

      if (homeConvexPol.isPointInside(projectedGoal2D))
      {
         System.out.println("-------------------------- Adding goal position");
         localGoal = projectedGoal2D;
      }

      localVisibilityGraph.createStaticVisibilityMap(localStart, localGoal);
      localVisibilityMap = localVisibilityGraph.getVisibilityMap();

      //      for (DefaultWeightedEdge edge : localVisibilityMap.edgeSet())
      //      {
      //         Point2D edgeSource = localVisibilityMap.getEdgeSource(edge);
      //         Point2D edgeTarget = localVisibilityMap.getEdgeTarget(edge);
      //         
      //         drawLine(basicJMEInterface.getZUpNode(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0),
      //                  ColorRGBA.Cyan, 3);
      //      }
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
      cluster.setClusterClosure(true);
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);
      cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));

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
         cluster.addRawPoint(pointToProject);
      }
   }

   private void createClustersFromRegions(PlanarRegion homeRegion, ArrayList<PlanarRegion> regions)
   {
      for (PlanarRegion region : lineObstacleRegions)
      {
         Cluster cluster = new Cluster();
         clusters.add(cluster);
         cluster.setType(Type.LINE);

         if (isRegionTooHighToStep(region, homeRegion))
         {
            cluster.setAdditionalExtrusionDistance(0);
         }
         else
         {
            cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
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

         LinearRegression linearRegression = new LinearRegression(points);
         linearRegression.calculateRegression();
         Point3D[] extremes = linearRegression.getTheTwoPointsFurthestApart();
         cluster.addRawPoint(extremes[0]);
         cluster.addRawPoint(extremes[1]);
         
         javaFXMultiColorMeshBuilder.addLine(extremes[0],
                                             extremes[1],
                                             0.005, Color.BLUE);
      }

      for (PlanarRegion region : polygonObstacleRegions)
      {
         Cluster cluster = new Cluster();
         clusters.add(cluster);
         cluster.setType(Type.POLYGON);
         cluster.setClusterClosure(true);

         Vector3D normal1 = calculateNormal(region);
         if (Math.abs(normal1.getZ()) >= 0.5)
         {
            cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
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
            cluster.addRawPoint(projectedPoint);
         }
      }

      for (Cluster cluster : clusters)
      {
         System.out.println("Created a cluster of type: " + cluster.getType() + " with " + cluster.getRawPointsInCluster().size() + " points");
      }
   }

   private ArrayList<PlanarRegion> determineWhichRegionsAreInside(PlanarRegion containingRegion, ArrayList<PlanarRegion> otherRegionsEx)
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

         //         DebugSphere sph = new DebugSphere(this, 0.05f, 30, 30, ColorRGBA.Blue);
         //         getZUpNode().attachChild(sph);
         //         sph.setLocalTranslation((float) transformedPt.getX32(), (float) transformedPt.getY32(), transformedPt.getZ32());

      }
      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

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

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPointFromOtherRegion = new Point3D();
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPointFromOtherRegion);

         //         DebugSphere sph = new DebugSphere(this, 0.05f, 30, 30, ColorRGBA.Orange);
         //         getZUpNode().attachChild(sph);
         //         sph.setLocalTranslation((float) projectedPointFromOtherRegion.getX32(), (float) projectedPointFromOtherRegion.getY32(), 0);

         if (homeConvexPol.isPointInside(new Point2D(projectedPointFromOtherRegion.getX(), projectedPointFromOtherRegion.getY())))
         {
            //            DebugSphere sph1 = new DebugSphere(this, 0.05f, 30, 30, ColorRGBA.Red);
            //            getZUpNode().attachChild(sph1);
            //            sph1.setLocalTranslation((float) projectedPointFromOtherRegion.getX32(), (float) projectedPointFromOtherRegion.getY32(), 0);
            return true;
            // regionsInside.add(otherRegion);
            // break;
         }
      }

      // Visuals
      //      DebugSphere sph = new DebugSphere(this, 0.25f, 30, 30, ColorRGBA.Red);
      //      getZUpNode().attachChild(sph);
      //      sph.setLocalTranslation((float) startingPosition.getX32(), (float) startingPosition.getY32(), 0);
      return false;
   }

   private void classifyExtrusion(PlanarRegion regionToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal(regionToProject);

      if (normal != null)
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

      System.out.println("Total obstacles to classify: " + regionsInsideHomeRegion.size() + "  Line obstacles: " + lineObstacleRegions.size()
            + "   Polygon obstacles: " + polygonObstacleRegions.size());

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
      FramePoint3D fpt = new FramePoint3D();
      fpt.set(point3D);
      RigidBodyTransform transToWorld = new RigidBodyTransform();
      regionToProjectTo.getTransformToWorld(transToWorld);
      fpt.applyTransform(transToWorld);

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