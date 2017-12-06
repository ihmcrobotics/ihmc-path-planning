package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2D observer, Point2D targetPoint, List<Point2D> listOfPointsInCluster)
   {
      for (int i = 0; i < listOfPointsInCluster.size() - 1; i++)
      {
         Point2D first = listOfPointsInCluster.get(i);
         Point2D second = listOfPointsInCluster.get(i + 1);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static ArrayList<Connection> getConnectionsThatAreInsideRegion(ArrayList<Connection> connections, PlanarRegion region)
   {
      ArrayList<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(new Point2D(connection.getSourcePoint()), new Point2D(connection.getTargetPoint()), region))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static ArrayList<Connection> getConnectionsThatAreInsideRegion(ArrayList<Connection> connections, List<Point2D> polygon)
   {
      ArrayList<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePolygon(new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY()),
                                                          new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY()), polygon))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static HashSet<Connection> createStaticVisibilityMap(Point2D start, Point2D goal, List<Cluster> clusters)
   {
      HashSet<Connection> connections = new HashSet<>();

      ArrayList<Point2D> listOfObserverPoints = new ArrayList<>();

      if (start != null)
      {
         listOfObserverPoints.add(start);
      }

      if (goal != null)
      {
         listOfObserverPoints.add(goal);
      }

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getNavigableExtrusionsInLocal())
            {
               listOfObserverPoints.add(point);
            }
         }
      }

      for (int i = 0; i < listOfObserverPoints.size(); i++)
      {
         Point2D observer = listOfObserverPoints.get(i);

         for (int j = i + 1; j < listOfObserverPoints.size(); j++)
         {
            Point2D target = listOfObserverPoints.get(j);

            if (observer.distance(target) > 0.01)
            {
               boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);

               if (targetIsVisible)
               {
                  connections.add(new Connection(new Point3D(observer), new Point3D(target)));
               }
            }
         }
      }

      return connections;
   }

   public static HashSet<Connection> createStaticVisibilityMap(Point2D observer, List<Cluster> clusters)
   {
      HashSet<Connection> connections = new HashSet<>();
      ArrayList<Point2D> listOfTargetPoints = new ArrayList<>();

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (!cluster.isDynamic())
         {
            for (Point2D point : cluster.getNavigableExtrusionsInLocal())
            {
               listOfTargetPoints.add(point);
            }
         }
      }

      for (int j = 0; j < listOfTargetPoints.size(); j++)
      {
         Point2D target = listOfTargetPoints.get(j);

         if (observer.distance(target) > 0.01)
         {
            boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);

            if (targetIsVisible)
            {
               connections.add(new Connection(new Point3D(observer), new Point3D(target)));
            }
         }
      }

      return connections;
   }

   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2D observer, Point2D targetPoint)
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

}
