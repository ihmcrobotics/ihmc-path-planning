package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ClusterManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;

public class VisibilityGraph
{
   private boolean DEBUG = true;
   private boolean PRINT_PERFORMANCE_STATS = false;

   protected ArrayList<Point2D> listOfObserverPoints = new ArrayList<>();

   private VisibilityMap visibilityMap;

   private ClusterManager clusterMgr;

   private long startTimeConnectionsCreation = System.currentTimeMillis();
   private long endTimeConnectionsCreation = System.currentTimeMillis();

   public VisibilityGraph(ClusterManager clusterMgr)
   {
      this.clusterMgr = clusterMgr;
      visibilityMap = new VisibilityMap();
   }

   HashSet<Connection> connections = new HashSet<>();

   public void createStaticVisibilityMap(Point2D start, Point2D goal)
   {
      startTimeConnectionsCreation = System.currentTimeMillis();
      listOfObserverPoints.clear();

      if (start != null)
      {
         listOfObserverPoints.add(start);
      }

      if (goal != null)
      {
         listOfObserverPoints.add(goal);
      }

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusterMgr.getClusters())
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getNavigableExtrusionsInLocal())
            {
               listOfObserverPoints.add(point);
            }
         }
      }
      startTimeConnectionsCreation = System.currentTimeMillis();

      for (int i = 0; i < listOfObserverPoints.size(); i++)
      {
         Point2D observer = listOfObserverPoints.get(i);

         for (int j = i + 1; j < listOfObserverPoints.size(); j++)
         {
            Point2D target = listOfObserverPoints.get(j);

            if (observer.distance(target) > 0.01)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(clusterMgr.getClusters(), observer, target);

               if (targetIsVisible)
               {
                  connections.add(new Connection(new Point3D(observer), new Point3D(target)));
               }
            }
         }
      }
      
      visibilityMap.setConnections(connections);

      endTimeConnectionsCreation = System.currentTimeMillis();

//      if (DEBUG)
//      {
//         System.out.println("Creating the map took " + (endTimeConnectionsCreation - startTimeConnectionsCreation) + " ms" + " with " + edges + " edges"
//               + " and " + staticVisibilityMap.vertexSet().size() + " vertices");
//      }
   }


   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters,Point2D observer, Point2D targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal()))
         {
            return false;
         }
      }

      return true;
   }

   public VisibilityMap getVisibilityMap()
   {
      return visibilityMap;
   }
}
