package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
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

   private JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;
   private double pathLength = 0.0;
   private Point3D startPos = new Point3D();
   private Point3D goalPos = new Point3D();
   private final VisibilityGraphsParameters parameters;

   private ArrayList<Connection> connectionPoints = new ArrayList<>();
   private ArrayList<Connection> globalMapPoints = new ArrayList<>();
   private ArrayList<DistancePoint> distancePoints = new ArrayList<>();

   public NavigableRegionsManager(VisibilityGraphsParameters parameters)
   {
      this(parameters, null, null);
   }
   
   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(regions, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      this(parameters, regions, null);
   }
   
   public NavigableRegionsManager(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(null, null, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(parameters, null, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(null, regions, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this.regions = regions;
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      ArrayList<PlanarRegion> regions1 = new ArrayList<>();
      regions1.add(regions.get(0));
      regions1.add(regions.get(1));
//      regions1.add(regions.get(2));
//      regions1.add(regions.get(3));
//      regions1.add(regions.get(4));
//      regions1.add(regions.get(5));
//      regions1.add(regions.get(6));
//    regions1.add(regions.get(20)); //slanted
//    regions1.add(regions.get(21)); //slanted 2
//    regions1.add(regions.get(24)); //slanted 3


      this.regions = regions;
   }

   public List<Point3D> calculateBodyPath(Point3D start, Point3D goal)
   {
      if(debug)
         PrintTools.info("Starting to calculate body path");
      
      long startBodyPathComputation = System.currentTimeMillis();
//                  start = new Point3D(10, 10, 0);
      //      goal = new Point3D(10, 10, 0);

//            goal = new Point3D(-3, -3, 0);

      long startCreatingMaps = System.currentTimeMillis();
      listOfLocalPlanners.clear();
      visMaps.clear();
      accesibleRegions.clear();

      this.startPos = start;
      this.goalPos = goal;
      classifyRegions(regions);
      
//      createVisibilityGraphForRegion(regions.get(0), startPos, goalPos);


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
      forceConnectionsOrSnapStartAndGoalIfNeeded(start, goal);
      long endForcingPoints = System.currentTimeMillis();

      long startGlobalMapTime = System.currentTimeMillis();
      createGlobalVisibilityGraph();
      long endGlobalMapTime = System.currentTimeMillis();

      long startSnappingTime = System.currentTimeMillis();
      Point3D goalPt = getSnappedPointFromMap(goalPos);
      Point3D startpt = getSnappedPointFromMap(startPos);
      long endSnappingTime = System.currentTimeMillis();

      long aStarStartTime = System.currentTimeMillis();
      
      List<Point3D> path = null;
      if (goalPt != null && startpt != null)
      {
         path = calculatePathOnVisibilityGraph(startpt, goalPt);
      }

      if (debug)
      {
         PrintTools.info("----Navigable Regions Manager Stats-----");
         PrintTools.info("Map creation completed in " + (endCreationTime - startCreatingMaps) + "ms");
         PrintTools.info("Connection completed in " + (endConnectingTime - startConnectingTime) + "ms");
         PrintTools.info("Forcing points took: " + (endForcingPoints - startForcingPoints) + "ms");
         PrintTools.info("Global Map creation took " + (endGlobalMapTime - startGlobalMapTime) + "ms");
         PrintTools.info("Snapping points took: " + (endSnappingTime - startSnappingTime) + "ms");
         PrintTools.info("A* took: " + (System.currentTimeMillis() - aStarStartTime) + "ms");
         PrintTools.info("Total time to find solution was: " + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
      }

      return path;
   }

   private List<Point3D> calculatePathOnVisibilityGraph(Point3D start, Point3D goal)
   {
      ArrayList<DefaultWeightedEdge> solution = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(globalVisMap, start, goal);
      return convertVisibilityGraphSolutionToPath(solution, start);
   }
   
   private List<Point3D> convertVisibilityGraphSolutionToPath(ArrayList<DefaultWeightedEdge> solution, Point3D start)
   {
      List<Point3D> path = new ArrayList<>();
      pathLength = 0.0;
      path.clear();
      
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

         if (!path.get(0).epsilonEquals(start, 1e-5))
         {
            Point3D pointOut = path.get(1);
            path.remove(1);
            path.add(0, pointOut);
         }
         
         if(debug)
            PrintTools.info("Visibility graph successfully found a solution");
      }
      else
      {
         if(debug)
            PrintTools.info("WARNING - Visibility graph found no solution");
      }
      
      return path;
   }

   private void forceConnectionsOrSnapStartAndGoalIfNeeded(Point3D start, Point3D goal)
   {
      //    System.out.println("Before forcing points global map has size: " + globalMapPoints.size());

      if (PlanarRegionTools.isPointInsideRegion(accesibleRegions, start))
      {
         if (isPointInsideNoGoZone(accesibleRegions, start))
         {
            start = snapConnectionToClosestPoint(start);
         }
      }
      else
      {
         forceConnectionToPoint(start);
      }

      if (PlanarRegionTools.isPointInsideRegion(accesibleRegions, goal))
      {
         if (isPointInsideNoGoZone(accesibleRegions, goal))
         {
            goal = snapConnectionToClosestPoint(goal);
         }
      }
      else
      {
         forceConnectionToPoint(goal);
      }

      startPos = start;
      goalPos = goal;
   }

   private void createGlobalVisibilityGraph()
   {
      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      int actualConnectionsAdded = 0;
      for (Connection pair : globalMapPoints)
      {
         Point3D pt1 = pair.getSourcePoint();
         Point3D pt2 = pair.getTargetPoint();
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
      //      System.out.println("Actual connections added: " + actualConnectionsAdded);
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
            globalMapPoints.add(new Connection(connection.getSourcePoint(), connection.getTargetPoint()));
            connectionsAdded++;
         }
      }

      //      System.out.println("Creating global map added " + connectionsAdded + " connections");
   }

   private Point3D snapConnectionToClosestPoint(Point3D position)
   {
      int connectionsAdded = 0;
      if (debug)
      {
         PrintTools.info("------>>>>  Point: " + position + " is inside a planar region and a No-Go-Zone - snapping connection");
      }

      distancePoints.clear();

      int index1 = 0;
      for (Connection pair : globalMapPoints)
      {
         DistancePoint point1 = new DistancePoint(pair.getSourcePoint(), pair.getSourcePoint().distance(position));
         DistancePoint point2 = new DistancePoint(pair.getTargetPoint(), pair.getTargetPoint().distance(position));

         distancePoints.add(point1);
         distancePoints.add(point2);
         index1++;
      }

      //      System.out.println("Added raw: " + index1);

      distancePoints.sort(new DistancePointComparator());

      ArrayList<Point3D> filteredList = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         //         System.out.println("Adding connection "+ point.point + "  to " + position);

         filteredList.add(point.point);
      }

      //      System.out.println("After filtering: " + filteredList.size());

      Iterator it = filteredList.iterator();

      Point3D pointToSnapTo = null;
      while (it.hasNext())
      {
         Point3D point = (Point3D) it.next();

         //Cannot add an edge where the source is equal to the target!
         if (!point.epsilonEquals(position, 1e-5))
         {
            pointToSnapTo = (Point3D) it.next();
            //            System.out.println("-----> new snapped point: " + pointToSnapTo);
            connectionPoints.add(new Connection(pointToSnapTo, position));
            break;
         }
      }

      return pointToSnapTo;
   }

   private void forceConnectionToPoint(Point3D position)
   {
      int connectionsAdded = 0;
      if (debug)
      {
         PrintTools.info("------>>>>  Point: " + position + " is not inside a planar region - forcing connection to closest points");
      }

      distancePoints.clear();

      int index1 = 0;
      for (Connection pair : globalMapPoints)
      {
         DistancePoint point1 = new DistancePoint(pair.getSourcePoint(), pair.getSourcePoint().distance(position));
         DistancePoint point2 = new DistancePoint(pair.getTargetPoint(), pair.getTargetPoint().distance(position));

         distancePoints.add(point1);
         distancePoints.add(point2);
         index1++;
      }

      //      System.out.println("Added raw: " + index1);

      distancePoints.sort(new DistancePointComparator());

      ArrayList<Point3D> filteredList = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         //         System.out.println("Adding connection "+ point.point + "  to " + position);

         filteredList.add(point.point);
      }

      //      System.out.println("After filtering: " + filteredList.size());

      Iterator it = filteredList.iterator();

      int index = 0;
      while (it.hasNext() && index < parameters.getNumberOfForcedConnections())
      {
         Point3D point = (Point3D) it.next();

         //Cannot add an edge where the source is equal to the target!
         if (!point.epsilonEquals(position, 1e-5))
         {
            globalMapPoints.add(new Connection(point, position));
            connectionPoints.add(new Connection(point, position));
            connectionsAdded++;
            index++;
            //            System.out.println("Adding connection "+ point + "  to " + position);
         }
      }

      //      System.out.println("Forcing connections added " + connectionsAdded + " connections");
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
         PrintTools.info("Starting connectivity check");
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
                        if (sourcePt.distance(targetPt) < parameters.getMinimumConnectionDistanceForRegions())
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

      //      System.out.println("Connecting maps added " + connectionsAdded + " connections");

   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      if (debug)
      {
         PrintTools.info("-----------Processing new region");
      }

      NavigableRegionLocalPlanner navigableRegionLocalPlanner = new NavigableRegionLocalPlanner(regions, region, startPos, goalPos, parameters);
      navigableRegionLocalPlanner.processRegion();
      listOfLocalPlanners.add(navigableRegionLocalPlanner);

      VisibilityMap localVisibilityMapInWorld = new VisibilityMap();
      visMaps.add(localVisibilityMapInWorld);

      for (Connection connection : navigableRegionLocalPlanner.getLocalVisibilityGraph().getConnections())
      {
         Point2D edgeSource = new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY());
         Point2D edgeTarget = new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY());

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
      int index = 0;
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
               index++;

               if (index > 1)
               {
                  if(debug)
                     PrintTools.info("POINT " + pointToCheck + " is inside a no-go zone!!!");
                  return true;
               }
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
            if (Math.abs(normal.getZ()) < parameters.getNormalZThresholdForAccessibleRegions())
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
