package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.scene.paint.Color;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   private final static boolean debug = false;

   private List<PlanarRegion> regions;
   private List<PlanarRegion> accesibleRegions = new ArrayList<>();
   private List<PlanarRegion> obstacleRegions = new ArrayList<>();
   private List<NavigableRegionLocalPlanner> listOfNavigableRegions = new ArrayList<>();
   private List<SimpleWeightedGraph<Point3D, DefaultWeightedEdge>> visMaps = new ArrayList<>();
   private SimpleWeightedGraph<Point3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

   private List<Point3D> path = new ArrayList<>();

   private JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   private double pathLength = 0.0;

   private Point3D startPos = new Point3D();
   private Point3D goalPos = new Point3D();
   
   double connectionthreshold = 0.2;

   ArrayList<DistancePoint> distancePoints = new ArrayList<>();

   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(regions, null);
   }

   public NavigableRegionsManager(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
   }

   public NavigableRegionsManager(List<PlanarRegion> regions, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this.regions = regions;
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
   }

   public void setPlanarRegions(ArrayList<PlanarRegion> regions)
   {
      this.regions = regions;
   }

   public List<Point3D> calculateBodyPath(Point3D start, Point3D goal)
   {
      this.startPos = start;
      this.goalPos = goal;
      classifyRegions(regions);

      for (PlanarRegion region : accesibleRegions)
      {
         createVisibilityGraphForRegion(region, startPos, goalPos);
      }

      for (NavigableRegionLocalPlanner navigableRegion : getListOfLocalPlanners())
      {
         if (navigableRegion.isPointInsideTheRegion(startPos))
         {
            System.out.println("Start point is inside a region");
            break;
         }
      }

      for (NavigableRegionLocalPlanner navigableRegion : getListOfLocalPlanners())
      {
         if (navigableRegion.isPointInsideTheRegion(goalPos))
         {
            System.out.println("Goal point is inside a region");
            break;
         }
      }

      connectLocalMaps();
      createGlobalMap();
      //forceConnectionToGoal(goalPos);
      //
      
      //Visualize
      for (SimpleWeightedGraph<Point3D, DefaultWeightedEdge> map : visMaps)
      {
         for (DefaultWeightedEdge edge : map.edgeSet())
         {
            Point3D pt1 = map.getEdgeSource(edge);
            Point3D pt2 = map.getEdgeTarget(edge);

            if (javaFXMultiColorMeshBuilder != null)
            {
               javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0052, Color.CYAN);
            }
         }
      }
      //

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

      //      System.out.println(startpt + "   " + goalpt);

      pathLength = 0.0;

      path.clear();

      Point3D lastPoint = null;
//      ArrayList<DefaultWeightedEdge> solution = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(globalVisMap, startpt, goalpt);
//      for (DefaultWeightedEdge edge : solution)
//      {
//         Point3D from = globalVisMap.getEdgeSource(edge);
//         Point3D to = globalVisMap.getEdgeTarget(edge);
//         pathLength = pathLength + from.distance(to);
//
//         if (!path.contains(new Point3D(from)))
//            path.add(from);
//         if (!path.contains(new Point3D(to)))
//            path.add(to);
//
//         //         javaFXMultiColorMeshBuilder.addLine(new Point3D(from.getX(), from.getY(), from.getZ()), new Point3D(to.getX(), to.getY(), to.getZ()), 0.025,
//         //                                             Color.RED);
//      }

      return path;
   }

   private void createGlobalMap()
   {
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
         }
      }
   }

   private void forceConnectionToGoal(Point3D goal)
   {
      Point3D test = null;

      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D pt1 = globalVisMap.getEdgeSource(edge);
         DistancePoint point = new DistancePoint(pt1, pt1.distance(goalPos));
         distancePoints.add(point);
      }

      distancePoints.sort(new DistancePointComparator());

      for (int i = 0; i < 5; i++)
      {
         DistancePoint dpt = distancePoints.get(i);
         javaFXMultiColorMeshBuilder.addSphere(0.025f, dpt.point, Color.BLUE);
         
         globalVisMap.addVertex(dpt.point);
         globalVisMap.addVertex(goal);
         
         DefaultWeightedEdge edge = new DefaultWeightedEdge();
         globalVisMap.addEdge(dpt.point, goal, edge);
         globalVisMap.setEdgeWeight(edge, dpt.point.distance(goal));
         
         javaFXMultiColorMeshBuilder.addLine(dpt.point, goal, 0.0052, Color.CYAN);
      }
   }

   private void connectLocalMaps()
   {
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

      if (debug)
      {
         System.out.println("Starting connectivity check");
      }

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

                        if (pt1.distance(pt2) < connectionthreshold)
                        {
                           globalVisMap.addVertex(pt1.getPoint());
                           globalVisMap.addVertex(pt2.getPoint());
                           DefaultWeightedEdge edge = new DefaultWeightedEdge();
                           globalVisMap.addEdge(pt1.getPoint(), pt2.getPoint(), edge);
                           globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));

                           if (javaFXMultiColorMeshBuilder != null)
                           {
                              javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0082, Color.YELLOW);
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      if (debug)
      {
         System.out.println("-----------Processing new region");
      }

      NavigableRegionLocalPlanner navigableRegionLocalPlanner = new NavigableRegionLocalPlanner(javaFXMultiColorMeshBuilder, regions, region, startPos, goalPos,
                                                                                                0.8);
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

   private void classifyRegions(List<PlanarRegion> regions)
   {
      Vector3D normal = new Vector3D();

      for (PlanarRegion region : regions)
      {
         if (!region.isEmpty())
         {
            region.getNormal(normal);
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

   public double computePathLength(double alpha)
   {
      return pathLength * (1.0 - alpha);
   }

   public double computePathLengthFromStart(double alpha)
   {
      return pathLength * (alpha);
   }

   public double computePathLength(int startWpIndex, int endWpIndex)
   {
      //      System.out.println(startWpIndex + "   " + endWpIndex);
      double pathLength = 0.0;
      for (int i = startWpIndex + 1; i <= endWpIndex; i++)
      {
         pathLength = pathLength + path.get(i - 1).distance(path.get(i));
      }

      return pathLength;
   }

   public Point3D getPointAlongPath(double alpha)
   {
      //      System.out.println("\n\nComplete Length: " + computePathLength(0.0));
      //      System.out.println("Desired length from the end: " + computePathLength(alpha));

      double tempPathDist = 0.0;
      double previousBeta = 0.0;

      int upperLimit = -1;
      int lowerLimit = -1;
      for (int i = 1; i < path.size(); i++)
      {
         Point3D from = path.get(i - 1);
         Point3D to = path.get(i);
         tempPathDist = tempPathDist + from.distance(to);

         double beta = tempPathDist / pathLength;

         if (alpha >= previousBeta && alpha <= beta)
         {
            upperLimit = i;
            lowerLimit = i - 1;

            if (javaFXMultiColorMeshBuilder != null)
            {
               javaFXMultiColorMeshBuilder.addSphere(0.045f, path.get(i - 1), Color.MAGENTA);
               javaFXMultiColorMeshBuilder.addSphere(0.045f, path.get(i), Color.MAGENTA);
            }

            break;
         }

         previousBeta = beta;
      }

      //      System.out.println("It is between " + (lowerLimit) + " and " + upperLimit);

      Vector3D vec = new Vector3D(path.get(upperLimit).getX() - path.get(lowerLimit).getX(), path.get(upperLimit).getY() - path.get(lowerLimit).getY(),
                                  path.get(upperLimit).getZ() - path.get(lowerLimit).getZ());
      vec.normalize();

      double pathFromStart = computePathLengthFromStart(alpha);
      //      System.out.println("Path from start: " + pathFromStart);
      //
      double pathFromStartToLowerLimit = computePathLength(0, lowerLimit);
      //      System.out.println("pathFromStartToLowerLimit: " + pathFromStartToLowerLimit);

      double amountOfRelativePath = pathFromStart - pathFromStartToLowerLimit;
      //      System.out.println(amountOfRelativePath + "   " + distanceBetweenLimits);

      Point3D newPoint = new Point3D(path.get(lowerLimit).getX() + vec.getX() * amountOfRelativePath,
                                     path.get(lowerLimit).getY() + vec.getY() * amountOfRelativePath,
                                     path.get(lowerLimit).getZ() + vec.getZ() * amountOfRelativePath);
      return newPoint;
   }

   public List<NavigableRegionLocalPlanner> getListOfLocalPlanners()
   {
      return listOfNavigableRegions;
   }

   public List<PlanarRegion> getListOfAccesibleRegions()
   {
      return accesibleRegions;
   }

   public List<PlanarRegion> getListOfObstacleRegions()
   {
      return obstacleRegions;
   }

   private class DistancePoint implements Comparable<DistancePoint>
   {
      Point3D point;
      double distance;

      public DistancePoint(Point3D point, double distance)
      {
         this.point = point;
         this.distance = distance;
      }

      @Override
      public int compareTo(DistancePoint point1)
      {
         if (distance > point1.distance)
         {
            return 1;
         }
         else if (distance < point1.distance)
         {
            return -1;
         }
         else
         {
            return 0;
         }
      }
   }

   private class DistancePointComparator implements Comparator<DistancePoint>
   {
      @Override
      public int compare(DistancePoint point1, DistancePoint point2)
      {
         return point1.compareTo(point2);
      }
   }
}
