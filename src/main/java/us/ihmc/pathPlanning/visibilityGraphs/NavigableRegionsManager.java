package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
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
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   private final static boolean debug = true;

   private List<PlanarRegion> regions;
   private List<PlanarRegion> accesibleRegions = new ArrayList<>();
   private List<PlanarRegion> obstacleRegions = new ArrayList<>();
   private List<NavigableRegionLocalPlanner> listOfLocalPlanners = new ArrayList<>();
   private List<VisibilityMap> visMaps = new ArrayList<>();
   private SimpleWeightedGraph<Point3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

   private List<Point3D> path = new ArrayList<>();
   private JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;
   private double pathLength = 0.0;
   private Point3D startPos = new Point3D();
   private Point3D goalPos = new Point3D();

   private ArrayList<Connection> connectionPoints = new ArrayList<>();
   private ArrayList<Connection> globalMapPoints = new ArrayList<>();
   private ArrayList<DistancePoint> distancePoints = new ArrayList<>();

   public NavigableRegionsManager()
   {
   }

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

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      this.regions = regions;
   }

   public List<Point3D> calculateBodyPath(Point3D start, Point3D goal)
   {
      long startBodyPathComputation = System.currentTimeMillis();
      //      start = new Point3D(10, 10, 0);
      //      goal = new Point3D(10, 10, 0);

      //      goal = new Point3D(-3, -3, 0);

      long startCreatingMaps = System.currentTimeMillis();
      listOfLocalPlanners.clear();
      visMaps.clear();
      accesibleRegions.clear();

      this.startPos = start;
      this.goalPos = goal;
      classifyRegions(regions);

      for (PlanarRegion region : accesibleRegions)
      {
         createVisibilityGraphForRegion(region, startPos, goalPos);
      }

      long endCreationTime = System.currentTimeMillis();

      connectionPoints.clear();
      globalMapPoints.clear();

      createGlobalMap();

      long startConnectingTime = System.currentTimeMillis();
      connectLocalMaps();
      long endConnectingTime = System.currentTimeMillis();

      long startForcingPoints = System.currentTimeMillis();

      if (!PlanarRegionTools.isPointInsideRegion(accesibleRegions, start))
      {
         forceConnectionToPoint(startPos);
      }
      else if (isPointInsideNoGoZone(accesibleRegions, start))
      {
         forceConnectionToPoint(startPos);
      }

      if (!PlanarRegionTools.isPointInsideRegion(accesibleRegions, goal))
      {
         forceConnectionToPoint(goalPos);
      }
      else if (isPointInsideNoGoZone(accesibleRegions, goalPos))
      {
         forceConnectionToPoint(goalPos);
      }

      long endForcingPoints = System.currentTimeMillis();

      System.out.println("So far global map has size: " + globalMapPoints.size());

      long startGlobalMapTime = System.currentTimeMillis();
      createVisibilityGraph();
      long endGlobalMapTime = System.currentTimeMillis();

      //      System.out.println(globalVisMap.containsVertex(globalMapPoints.get(globalMapPoints.size() - 1).point1));
      //      System.out.println(globalVisMap.containsVertex(globalMapPoints.get(globalMapPoints.size() - 1).point2));
      //      System.out.println(globalVisMap.containsEdge(globalMapPoints.get(globalMapPoints.size() - 1).point1,
      //                                                   globalMapPoints.get(globalMapPoints.size() - 1).point2));

      long startSnappingTime = System.currentTimeMillis();
      Point3D goalPt = getSnappedPointFromMap(goalPos);
      Point3D startpt = getSnappedPointFromMap(startPos);
      long endSnappingTime = System.currentTimeMillis();

      long aStarStartTime = System.currentTimeMillis();
      if (goalPt != null && startpt != null)
      {
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

      if (debug)
      {
         System.out.println("Map creation completed in " + (endCreationTime - startCreatingMaps) + "ms");
         System.out.println("Connection completed in " + (endConnectingTime - startConnectingTime) + "ms");
         System.out.println("Forcing points took: " + (endForcingPoints - startForcingPoints) + "ms");
         System.out.println("Global Map creation took " + (endGlobalMapTime - startGlobalMapTime) + "ms");
         System.out.println("Snapping points took: " + (endSnappingTime - startSnappingTime) + "ms");
         System.out.println("A* took: " + (System.currentTimeMillis() - aStarStartTime) + "ms");
         System.out.println("Total time to find solution was: " + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
      }

      return path;
   }

   private void createVisibilityGraph()
   {
      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      int actualConnectionsAdded = 0;
      for (Connection pair : globalMapPoints)
      {
         Point3D pt1 = pair.getPoint1();
         Point3D pt2 = pair.getPoint2();
         if (!pt1.epsilonEquals(pt2, 1e-5))
         {
            globalVisMap.addVertex(pt1);
            globalVisMap.addVertex(pt2);
            DefaultWeightedEdge edge = new DefaultWeightedEdge();
            globalVisMap.addEdge(pt1, pt2, edge);
            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
            actualConnectionsAdded++;
         }
      }
      System.out.println("Actual connections added: " + actualConnectionsAdded);
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
      int connectionsAdded = 0;
      for (VisibilityMap map : visMaps)
      {
         for (Connection connection : map.getConnections())
         {
            globalMapPoints.add(new Connection(connection.getPoint1(), connection.getPoint2()));
            connectionsAdded++;
         }
      }

      System.out.println("Creating global map added " + connectionsAdded + " connections");
   }

   private void forceConnectionToPoint(Point3D position)
   {
      int connectionsAdded = 0;
      if (debug)
      {
         System.out.println("Point: " + position + " is not inside a planar region - forcing connection");
      }

      distancePoints.clear();

      for (Connection pair : globalMapPoints)
      {
         DistancePoint point1 = new DistancePoint(pair.getPoint1(), pair.getPoint1().distance(position));
         DistancePoint point2 = new DistancePoint(pair.getPoint2(), pair.getPoint2().distance(position));

         distancePoints.add(point1);
         distancePoints.add(point2);
      }

      distancePoints.sort(new DistancePointComparator());

      HashSet<Point3D> filteredList = new HashSet<>();

      for (DistancePoint point : distancePoints)
      {
         filteredList.add(point.point);
      }

      //      ArrayList<Point3D> filteredList = VisibilityTools.removeDuplicatePoints(newList);

      Iterator it = filteredList.iterator();

      int index = 0;
      while (it.hasNext() && index < VisibilityGraphsParameters.NUMBER_OF_FORCED_CONNECTIONS)
      {
         Point3D point = (Point3D) it.next();
         DefaultWeightedEdge edge = new DefaultWeightedEdge();

         //Cannot add an edge where the source is equal to the target!
         if (!point.epsilonEquals(position, 1e-5))
         {
            globalMapPoints.add(new Connection(point, position));
            connectionPoints.add(new Connection(point, position));
            connectionsAdded++;
            index++;
         }
      }

      System.out.println("Forcing connections added " + connectionsAdded + " connections");
   }

   private void connectLocalMaps()
   {
      //      int mapIndex = 0;
      //      if (listOfLocalPlanners.size() > 1)
      //      {
      //         //         System.out.println("# of navigable regions: " + listOfNavigableRegions.size());
      //         for (NavigableRegionLocalPlanner sourceLocalRegion : listOfLocalPlanners)
      //         {
      //            //            System.out.println("Map " + mapIndex + " has " + sourceLocalRegion.getLocalVisibilityGraph().edgeSet().size() + " connections");
      //            mapIndex++;
      //         }
      //      }

      int connectionsAdded = 0;
      if (debug)
      {
         System.out.println("Starting connectivity check");
      }

      if (listOfLocalPlanners.size() > 1)
      {
         for (VisibilityMap sourceMap : visMaps)
         {
            HashSet<Point3D> sourcePoints = sourceMap.getVertices();

            for (Point3D sourcePt : sourcePoints)
            {
               for (VisibilityMap targetMap : visMaps)
               {
                  if (sourceMap != targetMap)
                  {
                     HashSet<Point3D> targetPoints = targetMap.getVertices();

                     for (Point3D targetPt : targetPoints)
                     {
                        if (sourcePt.distance(targetPt) < VisibilityGraphsParameters.MIN_CONNECTION_DISTANCE_FOR_REGIONS)
                        {
                           connectionPoints.add(new Connection(sourcePt, targetPt));
                           globalMapPoints.add(new Connection(sourcePt, targetPt));
                           connectionsAdded++;
                        }
                     }
                  }
               }
            }
         }
      }

      System.out.println("Connecting maps added " + connectionsAdded + " connections");

   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      if (debug)
      {
         System.out.println("-----------Processing new region");
      }

      NavigableRegionLocalPlanner navigableRegionLocalPlanner = new NavigableRegionLocalPlanner(regions, region, startPos, goalPos,
                                                                                                VisibilityGraphsParameters.EXTRUSION_DISTANCE);
      navigableRegionLocalPlanner.processRegion();
      listOfLocalPlanners.add(navigableRegionLocalPlanner);

      VisibilityMap localVisibilityMapInWorld = new VisibilityMap();
      visMaps.add(localVisibilityMapInWorld);

      for (Connection connection : navigableRegionLocalPlanner.getLocalVisibilityGraph().getConnections())
      {
         Point2D edgeSource = new Point2D(connection.getPoint1().getX(), connection.getPoint1().getY());
         Point2D edgeTarget = new Point2D(connection.getPoint2().getX(), connection.getPoint2().getY());

         FramePoint3D pt1 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0));
         pt1.changeFrame(ReferenceFrame.getWorldFrame());
         FramePoint3D pt2 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0));
         pt2.changeFrame(ReferenceFrame.getWorldFrame());

         localVisibilityMapInWorld.addConnection(new Connection(pt1.getPoint(), pt2.getPoint()));
      }

      localVisibilityMapInWorld.computeVertices();
   }



   public boolean isPointInsideNoGoZone(List<PlanarRegion> regions, Point3D pointToCheck)
   {
      for (NavigableRegionLocalPlanner localPlanner : listOfLocalPlanners)
      {
         for (Cluster cluster : localPlanner.getClusters())
         {
            Point2D[] homePointsArr = new Point2D[cluster.getNonNavigableExtrusionsInWorld().size()];

            for (int i = 0; i < cluster.getNonNavigableExtrusionsInWorld().size(); i++)
            {
               Point3D extrusion = cluster.getNonNavigableExtrusionsInWorld().get(i);
               Point2D point2D = new Point2D(extrusion.getX(), extrusion.getY());
               homePointsArr[i] = point2D;
            }

            ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);

            if (homeConvexPol.isPointInside(new Point2D(pointToCheck.getX(), pointToCheck.getY())))
            {
               System.out.println("POINT " + pointToCheck + " is inside a no-go zone!!!");
               return true;
            }
         }
      }

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
            if (Math.abs(normal.getZ()) < VisibilityGraphsParameters.NORMAL_Z_THRESHOLD_FOR_ACCESIBLE_REGIONS)
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


   public List<NavigableRegionLocalPlanner> getListOfLocalPlanners()
   {
      return listOfLocalPlanners;
   }

   public List<PlanarRegion> getListOfAccesibleRegions()
   {
      return accesibleRegions;
   }

   public List<PlanarRegion> getListOfObstacleRegions()
   {
      return obstacleRegions;
   }

   public ArrayList<Connection> getGlobalMapPoints()
   {
      return globalMapPoints;
   }

   public ArrayList<Connection> getConnectionPoints()
   {
      return connectionPoints;
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
