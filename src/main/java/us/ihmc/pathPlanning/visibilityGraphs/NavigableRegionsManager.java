package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   double connectionthreshold = 0.1;

   ArrayList<Connection> points = new ArrayList<>();

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
//      start = new Point3D(10,10,0);
//      goal = new Point3D(10,10,0);
      
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
            //            System.out.println("Start point is inside a region");
            break;
         }
      }

      for (NavigableRegionLocalPlanner navigableRegion : getListOfLocalPlanners())
      {
         if (navigableRegion.isPointInsideTheRegion(goalPos))
         {
            //            System.out.println("Goal point is inside a region");
            break;
         }
      }

      points.clear();
      connectLocalMaps();
      createGlobalMap();

      if (!isPointInsideRegion(accesibleRegions, start))
      {
         forceConnectionToPoint(startPos);
      }

      if (!isPointInsideRegion(accesibleRegions, goal))
      {
         forceConnectionToPoint(goalPos);
      }

      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      for (Connection pair : points)
      {
         Point3D pt1 = pair.point1;
         Point3D pt2 = pair.point2;
         if (!pt1.epsilonEquals(pt2, 1e-5))
         {
            globalVisMap.addVertex(pt1);
            globalVisMap.addVertex(pt2);
            DefaultWeightedEdge edge = new DefaultWeightedEdge();
            globalVisMap.addEdge(pt1, pt2, edge);
            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
         }
      }

      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D pt1 = globalVisMap.getEdgeSource(edge);
         Point3D pt2 = globalVisMap.getEdgeTarget(edge);

         if (javaFXMultiColorMeshBuilder != null)
         {
            javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0052, Color.CYAN);
         }
      }

      Point3D goalPt = getSnappedPointFromMap(goalPos);
      Point3D startpt = getSnappedPointFromMap(startPos);

      if (goalPt != null && startpt != null)
      {
         ArrayList<DefaultWeightedEdge> edgess = new ArrayList<>();
         edgess.addAll(globalVisMap.edgeSet());

         pathLength = 0.0;

         path.clear();

         ArrayList<DefaultWeightedEdge> solution = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(globalVisMap, startpt, goalPt);

         if (solution != null)
         {
            for (DefaultWeightedEdge edge : solution)
            {
               Point3D from = globalVisMap.getEdgeSource(edge);
               Point3D to = globalVisMap.getEdgeTarget(edge);
               pathLength = pathLength + from.distance(to);

               if (!path.contains(new Point3D(from)))
                  path.add(from);
               if (!path.contains(new Point3D(to)))
                  path.add(to);
            }

            if (!path.get(0).epsilonEquals(startpt, 1e-5))
            {
               Point3D pointOut = path.get(1);
               path.remove(1);
               path.add(0, pointOut);
            }
         }
         else
         {
            System.out.println("WARNING - NO SOLUTION WAS FOUND!");
         }
      }

      return path;
   }

   public Point3D getSnappedPointFromMap(Point3D position)
   {
      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D source = globalVisMap.getEdgeSource(edge);

         if (Math.abs(source.getX() - position.getX()) < 0.001)
         {
            if (Math.abs(source.getY() - position.getY()) < 0.001)
            {
               if (Math.abs(source.getZ() - position.getZ()) < 0.001)
               {
                  return source;
               }
            }
         }

         Point3D target = globalVisMap.getEdgeTarget(edge);

         if (Math.abs(target.getX() - position.getX()) < 0.001)
         {
            if (Math.abs(target.getY() - position.getY()) < 0.001)
            {
               if (Math.abs(target.getZ() - position.getZ()) < 0.001)
               {
                  return target;
               }
            }
         }
      }

      return null;
   }

   private void createGlobalMap()
   {
      for (SimpleWeightedGraph<Point3D, DefaultWeightedEdge> map : visMaps)
      {
         for (DefaultWeightedEdge edge : map.edgeSet())
         {
            Point3D pt1 = map.getEdgeSource(edge);
            Point3D pt2 = map.getEdgeTarget(edge);

            points.add(new Connection(pt1, pt2));

            //            globalVisMap.addVertex(pt1);
            //            globalVisMap.addVertex(pt2);
            //            globalVisMap.addEdge(pt1, pt2, edge);
            //            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
         }
      }
   }

   private void forceConnectionToPoint(Point3D position)
   {
      if(debug)
      {
         System.out.println("Point: " + position + " is not inside a planar region - forcing connection");
      }
      
      Point3D tempPoint = null;
      distancePoints.clear();

      for (Connection pair : points)
      {
         DistancePoint point1 = new DistancePoint(pair.point1, pair.point1.distance(position));
         DistancePoint point2 = new DistancePoint(pair.point2, pair.point2.distance(position));

         distancePoints.add(point1);
         distancePoints.add(point2);
      }

      distancePoints.sort(new DistancePointComparator());

      for (int i = 0; i < 5; i++)
      {
         DistancePoint dpt = distancePoints.get(i);
         tempPoint = dpt.point;
         if (javaFXMultiColorMeshBuilder != null)
            javaFXMultiColorMeshBuilder.addSphere(0.025f, dpt.point, Color.BLUE);

         points.add(new Connection(dpt.point, position));

         DefaultWeightedEdge edge = new DefaultWeightedEdge();

         //Cannot add an edge where the source is equal to the target!
         if (!dpt.point.epsilonEquals(position, 1e-5))
         {
            points.add(new Connection(dpt.point, position));
         }
      }
   }

   private void connectLocalMaps()
   {

      int mapIndex = 0;
      if (listOfNavigableRegions.size() > 1)
      {
         //         System.out.println("# of navigable regions: " + listOfNavigableRegions.size());
         for (NavigableRegionLocalPlanner sourceLocalRegion : listOfNavigableRegions)
         {
            //            System.out.println("Map " + mapIndex + " has " + sourceLocalRegion.getLocalVisibilityGraph().edgeSet().size() + " connections");
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
                           points.add(new Connection(pt1.getPoint(), pt2.getPoint()));

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

   public boolean isPointInsideRegion(List<PlanarRegion> regions, Point3D point)
   {
      for (PlanarRegion region : regions)
      {
         Point2D[] homePointsArr = new Point2D[region.getConvexHull().getNumberOfVertices()];
         for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
         {
            Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
            Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
            FramePoint3D fpt = new FramePoint3D();
            fpt.set(point3D);
            RigidBodyTransform transToWorld = new RigidBodyTransform();
            region.getTransformToWorld(transToWorld);
            fpt.applyTransform(transToWorld);
            Point3D transformedPt = fpt.getPoint();

            homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
         }

         ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
         homeConvexPol.update();

         if (homeConvexPol.isPointInside(new Point2D(point.getX(), point.getY())))
         {
            System.out.println("POINT" + point + " IS INSIDE A REGION");
            return true;
         }
      }
      System.out.println("POINT" + point + " IS NOT INSIDE A REGION");

      return false;
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

   public ArrayList<Connection> getPoints()
   {
      return points;
   }

   public void setPoints(ArrayList<Connection> points)
   {
      this.points = points;
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
