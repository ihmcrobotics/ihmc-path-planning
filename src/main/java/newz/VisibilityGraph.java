package newz;

import java.util.ArrayList;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import tools.VisibilityTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class VisibilityGraph
{
   private boolean DEBUG = false;
   private boolean PRINT_PERFORMANCE_STATS = false;

   protected ArrayList<Point2D> listOfObserverPoints = new ArrayList<>();
   private static double epsilon = 0.00001;
   protected ArrayList<Point2D> listOfTargetsIncludingDynamicObjects = new ArrayList<>();

   boolean configurationSpaceIsInitialized = false;
   protected boolean enableObstacleAvoidance = false;

   protected SimpleWeightedGraph<Point2D, DefaultWeightedEdge> staticVisibilityMap;

   ClusterMgr clusterMgr;
   
   long startTimeConnectionsCreation = System.currentTimeMillis();
   long endTimeConnectionsCreation = System.currentTimeMillis();

   public VisibilityGraph(ClusterMgr clusterMgr)
   {
      this.clusterMgr = clusterMgr;
      staticVisibilityMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
   }
   

   public ArrayList<Point2D> solve(Point2D start, Point2D goal)
   {
      if (start.distance(goal) < 1E-1)
      {
         ArrayList<Point2D> path = new ArrayList<>();
         path.add(start);

         return path;
      }

      try
      {
         //       System.out.println("Started calculating route...");
         long startTimePathCalculation = System.currentTimeMillis();
         ArrayList<DefaultWeightedEdge> edges = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(staticVisibilityMap, start, goal);
         ArrayList<Point2D> path = convertEdgesToPath(staticVisibilityMap, edges, start, goal);
         long endTimePathCalculation = System.currentTimeMillis();

         if (PRINT_PERFORMANCE_STATS)
         {
            System.out.println("\n--- Visisibility Graph Performance Stats ---");
            System.out.println("Creating the map took " + (endTimeConnectionsCreation - startTimeConnectionsCreation) + " ms");
            System.out.println("Calculating route took " + (endTimePathCalculation - startTimePathCalculation) + " ms");
         }

         return path;
      }
      catch (IllegalArgumentException arg)
      {
         return null;
      }
   }

   public static ArrayList<Point2D> convertEdgesToPath(SimpleWeightedGraph<Point2D, DefaultWeightedEdge> visibilityMap, ArrayList<DefaultWeightedEdge> edges,
                                                 Point2D start, Point2D goal)
   {
      ArrayList<Point2D> path = new ArrayList<>();
      Point2D lastNode = start;
      path.add(lastNode);

      for (DefaultWeightedEdge edge : edges)
      {
         Point2D edgeSource = visibilityMap.getEdgeSource(edge);
         Point2D edgeTarget = visibilityMap.getEdgeTarget(edge);
         
         if (edgeSource.epsilonEquals(lastNode, epsilon))
         {
            path.add(edgeTarget);
            lastNode = edgeTarget;
         }
         else if (edgeTarget.epsilonEquals(lastNode, epsilon))
         {
            path.add(edgeSource);
            lastNode = edgeSource;
         }
         else
         {
            System.err.println(lastNode + " did not equal source " + edgeSource + " or target " + edgeTarget);
         }
      }

      if (!path.get(path.size() - 1).epsilonEquals(new Point2D(goal.getX(), goal.getY()), epsilon))
      {
         System.err.println("last node " + path.get(path.size() - 1) + " does not match the goal " + goal);
      }

      return path;
   }


   public void createStaticVisibilityMap(Point2D start, Point2D goal)
   {
      startTimeConnectionsCreation = System.currentTimeMillis();
      listOfObserverPoints.clear();
      
      if(start != null)
      {
         listOfObserverPoints.add(start);
      }
      
      if(goal != null)
      {
         listOfObserverPoints.add(goal);
      }

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusterMgr.getClusters())
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getUpdatedNavigableExtrusions())
            {
               listOfObserverPoints.add(point);
            }
         }
      }

      int edges = 0;
      // Add new connections
      for (Point2D observer : listOfObserverPoints)
      {
         for (Point2D target : listOfObserverPoints)
         {
            if (observer.distance(target) > 0.01)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(observer, target);
               
               if (targetIsVisible)
               {
                  staticVisibilityMap.addVertex(observer);
                  staticVisibilityMap.addVertex(target);

                  DefaultWeightedEdge visibilityEdge = staticVisibilityMap.addEdge(observer, target);
                  edges++;

                  // These are undirected edges. Therefore we cannot add duplicate or reversed edges. null indicates adding a duplicate
                  if (visibilityEdge != null)
                  {
                     double distanceBetweenVertices = observer.distance(target);
                     staticVisibilityMap.setEdgeWeight(visibilityEdge, distanceBetweenVertices);
                  }
               }
            }
         }
      }
      
      endTimeConnectionsCreation = System.currentTimeMillis();
      
      System.out.println("Creating the map took " + (endTimeConnectionsCreation - startTimeConnectionsCreation) + " ms" + " with " + edges + " edges");
   }

   public boolean isPointVisibleForStaticMaps(Point2D observer, Point2D targetPoint)
   {
      for (Cluster cluster : clusterMgr.getClusters())
      {
         if(!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getListOfNonNavigableExtrusions()))
         {
            return false;
         }
      }

      return true;
   }

   public SimpleWeightedGraph<Point2D, DefaultWeightedEdge> getVisibilityMap()
   { 
      return staticVisibilityMap;
   }
}
