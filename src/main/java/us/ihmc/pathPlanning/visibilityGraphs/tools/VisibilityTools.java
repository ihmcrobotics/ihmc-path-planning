package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
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

         if (PlanarRegionTools.areBothPointsInsidePolygon(new Point2D(connection.getSourcePoint()),
                                 new Point2D(connection.getTargetPoint()), region))
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

}
