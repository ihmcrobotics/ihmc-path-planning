package us.ihmc.pathPlanning.workingExamples;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.clusterManagement.Cluster;
import us.ihmc.pathPlanning.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionLocalPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraph;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_SpiralStairCase extends Application
{
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   ArrayList<PlanarRegion> accesibleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> obstacleRegions = new ArrayList<>();
   ArrayList<SimpleWeightedGraph<Point3D, DefaultWeightedEdge>> visMaps = new ArrayList<>();
   ArrayList<NavigableRegionLocalPlanner> listOfNavigableRegions = new ArrayList<>();

   SimpleWeightedGraph<Point3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   Point3D startPos = new Point3D(0.2, 1.25, 0);
   Point3D goalPos = new Point3D(-0.8, 1, 1);
   
   long startTime = 0;
   long startLocalMapsTime = 0;
   long timeLocalMaps = 0;
   long startConnectivityTime = 0;
   long timeConnectivity = 0;
   long startTimeForGlobalMapCreation = 0;
   long timeForGlobalMapCreation = 0;

   public Example_SpiralStairCase()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      regions = PointCloudTools.loadPlanarRegionsFromFile("Data/PlanarRegions_SpiralStairCase.txt");
      
      startTime = System.currentTimeMillis();
      
      classifyRegions(regions);

      //      "PlanarRegions_DoubleRamp.txt"
      //      Point3D startPos = new Point3D(4.6, 3.6, 0);
      //      Point3D goalPos = new Point3D(1, 2.5, 1);

      //      "PlanarRegions_Ramp_OneSided.txt"
      //      Point3D startPos = new Point3D(1, -1.5, 0);
      //      Point3D goalPos = new Point3D(4.5, 0, 1);

      //      "PlanarRegions_LargeMixedCourse.txt"
      //      Point3D startPos = new Point3D(0, 3.5, 0); //-0.7, 3.9, 0 with region 31
      //      Point3D goalPos = new Point3D(0.4, 4.7, 0); // on region 34

      //    Point3D startPos = new Point3D(0, 0, 0); //-0.7, 3.9, 0 with region 0
      //    Point3D goalPos = new Point3D(0.4, 4.7, 0); // on region 34

      startPos = projectPointToPlane(startPos, regions.get(0));
      goalPos = projectPointToPlane(goalPos, regions.get(9));

      javaFXMultiColorMeshBuilder.addSphere(0.03f, startPos, Color.GREEN);
      javaFXMultiColorMeshBuilder.addSphere(0.03f, goalPos, Color.RED);
      
      startLocalMapsTime = System.currentTimeMillis();

      //      accesibleRegions.remove(1);
      //      accesibleRegions.remove(2);
      //      accesibleRegions.remove(2);
      //      accesibleRegions.remove(2);

      //            createVisibilityGraphForRegion(regions.get(0), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(1), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(2), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(3), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(4), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(5), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(6), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(7), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(8), startPos, goalPos);
      createVisibilityGraphForRegion(regions.get(9), startPos, goalPos);
      
      //      Random rand = new Random();
      //      for (PlanarRegion region : regions)
      //      {
      //         int id = rand.nextInt();
      //         region.setRegionId(id);
      //
      //         RigidBodyTransform transform = new RigidBodyTransform();
      //         region.getTransformToWorld(transform);
      //
      //         Color color = getRegionColor(region.getRegionId());
      //         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
      //         {
      //            javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
      //         }
      //      }

      for (PlanarRegion region : accesibleRegions)
      {
         visualizeRegion(region, Color.GREEN);
      }

      for (PlanarRegion region : obstacleRegions)
      {
         visualizeRegion(region, Color.RED);
      }

      //      for (PlanarRegion region : accesibleRegions)
      //      {
      //         createVisibilityGraphForRegion(region, startPos, goalPos);
      //      }
      
      timeLocalMaps = System.currentTimeMillis() - startLocalMapsTime;

      connectLocalMaps();
      createGlobalMap();

      Point3D goalpt = null;

      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D pt1 = globalVisMap.getEdgeSource(edge);

         if (Math.abs(pt1.getX() - goalPos.getX()) < 0.001)
         {
            if (Math.abs(pt1.getY() - goalPos.getY()) < 0.001)
            {
               if (Math.abs(pt1.getZ() - goalPos.getZ()) < 0.001)
               {
                  goalpt = globalVisMap.getEdgeSource(edge);
               }
            }
         }
      }

      Point3D startpt = null;

      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D pt1 = globalVisMap.getEdgeSource(edge);

         if (Math.abs(pt1.getX() - startPos.getX()) < 0.001)
         {
            if (Math.abs(pt1.getY() - startPos.getY()) < 0.001)
            {
               if (Math.abs(pt1.getZ() - startPos.getZ()) < 0.001)
               {
                  startpt = globalVisMap.getEdgeSource(edge);
               }
            }
         }
      }

      ArrayList<DefaultWeightedEdge> edgess = new ArrayList<>();
      edgess.addAll(globalVisMap.edgeSet());

      ArrayList<DefaultWeightedEdge> solution = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(globalVisMap, startpt, goalpt);
      for (DefaultWeightedEdge edge : solution)
      {
         Point3D from = globalVisMap.getEdgeSource(edge);
         Point3D to = globalVisMap.getEdgeTarget(edge);
         javaFXMultiColorMeshBuilder.addLine(new Point3D(from.getX(), from.getY(), from.getZ()), new Point3D(to.getX(), to.getY(), to.getZ()), 0.025,
                                             Color.RED);
      }
      
      System.out.println("---- Stats ----");
      System.out.println("Complete Planning process:" + (System.currentTimeMillis() - startTime) + " ms");
      System.out.println("Local maps: " + timeLocalMaps + " ms");
      System.out.println("Vis. Network connectivity: " + timeConnectivity + " ms");
      System.out.println("Global map creation: " + timeForGlobalMapCreation + " ms");

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public void visualize()
   {

   }

   public void visualizeRegion(PlanarRegion region, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      region.getTransformToWorld(transform);

      for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
      {
         javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
      }
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      System.out.println("-----------Processing new region");
      NavigableRegionLocalPlanner navigableRegionLocalPlanner = new NavigableRegionLocalPlanner(javaFXMultiColorMeshBuilder, regions, region, startPos,
                                                                                                goalPos);
      navigableRegionLocalPlanner.processRegion();
      listOfNavigableRegions.add(navigableRegionLocalPlanner);

      SimpleWeightedGraph<Point3D, DefaultWeightedEdge> localRegionVisMapInWorld = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
      visMaps.add(localRegionVisMapInWorld);

      for (DefaultWeightedEdge edge : navigableRegionLocalPlanner.getLocalVisibilityGraph().edgeSet())
      {
         Point2D edgeSource = navigableRegionLocalPlanner.getLocalVisibilityGraph().getEdgeSource(edge);
         Point2D edgeTarget = navigableRegionLocalPlanner.getLocalVisibilityGraph().getEdgeTarget(edge);

         FramePoint3D pt1 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0));
         pt1.changeFrame(ReferenceFrame.getWorldFrame());
         FramePoint3D pt2 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0));
         pt2.changeFrame(ReferenceFrame.getWorldFrame());

         localRegionVisMapInWorld.addVertex(pt1.getPoint());
         localRegionVisMapInWorld.addVertex(pt2.getPoint());
         DefaultWeightedEdge tempEdge = new DefaultWeightedEdge();
         localRegionVisMapInWorld.addEdge(pt1.getPoint(), pt2.getPoint(), tempEdge);
         localRegionVisMapInWorld.setEdgeWeight(edge, pt1.distance(pt2));

      }
   }

   private void connectLocalMaps()
   {
      startConnectivityTime = System.currentTimeMillis();
      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      int mapIndex = 0;
      if (listOfNavigableRegions.size() > 1)
      {
         System.out.println("# of navigable regions: " + listOfNavigableRegions.size());
         for (NavigableRegionLocalPlanner sourceLocalRegion : listOfNavigableRegions)
         {
            System.out.println("Map " + mapIndex + " has " + sourceLocalRegion.getLocalVisibilityGraph().edgeSet().size() + " connections");
            mapIndex++;
         }
      }

      System.out.println("Starting connectivity check");

      if (listOfNavigableRegions.size() > 1)
      {
         for (NavigableRegionLocalPlanner sourceLocalRegion : listOfNavigableRegions)
         {
            for (Point2D sourcePt : sourceLocalRegion.getLocalVisibilityGraph().vertexSet())
            {
               for (NavigableRegionLocalPlanner targetLocalRegion : listOfNavigableRegions)
               {
                  if (targetLocalRegion != sourceLocalRegion)
                  {
                     for (Point2D targetPt : targetLocalRegion.getLocalVisibilityGraph().vertexSet())
                     {
                        FramePoint3D pt1 = new FramePoint3D(sourceLocalRegion.getLocalReferenceFrame(), new Point3D(sourcePt.getX(), sourcePt.getY(), 0));
                        pt1.changeFrame(ReferenceFrame.getWorldFrame());

                        FramePoint3D pt2 = new FramePoint3D(targetLocalRegion.getLocalReferenceFrame(), new Point3D(targetPt.getX(), targetPt.getY(), 0));
                        pt2.changeFrame(ReferenceFrame.getWorldFrame());
                        //                        javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0082, Color.RED);

                        if (pt1.distance(pt2) < 0.2)
                        {
                           globalVisMap.addVertex(pt1.getPoint());
                           globalVisMap.addVertex(pt2.getPoint());
                           DefaultWeightedEdge edge = new DefaultWeightedEdge();
                           globalVisMap.addEdge(pt1.getPoint(), pt2.getPoint(), edge);
                           globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
                           javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0082, Color.YELLOW);
                        }
                     }
                  }
               }
            }
         }
      }
      
      timeConnectivity = System.currentTimeMillis() - startConnectivityTime;
   }

   private void createGlobalMap()
   {
      startTimeForGlobalMapCreation = System.currentTimeMillis();
      for (int i = 0; i < visMaps.size(); i++)
      {
         SimpleWeightedGraph<Point3D, DefaultWeightedEdge> sourceMap = visMaps.get(i);

         for (int j = 0; j < visMaps.size(); j++)
         {
            if (i == j)
            {
               continue;
            }

            SimpleWeightedGraph<Point3D, DefaultWeightedEdge> targetMap = visMaps.get(j);

            for (DefaultWeightedEdge edge : sourceMap.edgeSet())
            {
               Point3D pt1 = sourceMap.getEdgeSource(edge);
               for (DefaultWeightedEdge edge1 : targetMap.edgeSet())
               {
                  Point3D pt2 = targetMap.getEdgeSource(edge1);

                  if (pt1.distance(pt2) < 0.3)
                  {
                     globalVisMap.addVertex(pt1);
                     globalVisMap.addVertex(pt2);
                     globalVisMap.addEdge(pt1, pt2, edge);
                     globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
                     //                     javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0052, Color.RED);
                  }
               }
            }
         }
      }

      for (SimpleWeightedGraph<Point3D, DefaultWeightedEdge> map : visMaps)
      {
         for (DefaultWeightedEdge edge : map.edgeSet())
         {
            Point3D pt1 = map.getEdgeSource(edge);
            Point3D pt2 = map.getEdgeTarget(edge);

            globalVisMap.addVertex(pt1);
            globalVisMap.addVertex(pt2);
            globalVisMap.addEdge(pt1, pt2, edge);
            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));

            javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0052, Color.CYAN);

         }
      }
      
      timeForGlobalMapCreation = System.currentTimeMillis() - startTimeForGlobalMapCreation;
   }

   private void classifyRegions(ArrayList<PlanarRegion> regions)
   {
      for (PlanarRegion region : regions)
      {
         Vector3D normal = calculateNormal(region);

         if (normal != null)
         {
            if (Math.abs(normal.getZ()) < 0.8)
            {
               obstacleRegions.add(region);
            }
            else
            {
               accesibleRegions.add(region);
            }
         }
      }
   }

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

   private Vector3D calculateNormal1(PlanarRegion region)
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

   public Point3D projectPointToPlane(Point3D pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal1(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);

      FramePoint3D fpt1 = new FramePoint3D();
      fpt1.set(point3D);
      RigidBodyTransform trans = new RigidBodyTransform();
      regionToProjectTo.getTransformToWorld(trans);
      fpt1.applyTransform(trans);

      Point3D projectedPoint = new Point3D();
      if (!EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, fpt1.getPoint(), normal, projectedPoint))
      {
         projectedPoint = null;
      }
      return projectedPoint;
   }

   public static void main(String[] args)
   {
      launch();
   }
}