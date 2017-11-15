package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.HashSet;

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

      for (Point2D observer : listOfObserverPoints)
      {
         for (Point2D target : listOfObserverPoints)
         {
            if (observer.distance(target) > 0.01)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(observer, target);

               if (targetIsVisible)
               {
                  connections.add(new Connection(new Point3D(observer.getX(), observer.getY(), 0), new Point3D(target.getX(), target.getY(), 0)));
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


   public boolean isPointVisibleForStaticMaps(Point2D observer, Point2D targetPoint)
   {
      for (Cluster cluster : clusterMgr.getClusters())
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
