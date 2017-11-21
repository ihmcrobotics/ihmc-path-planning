package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2D observer, Point2D targetPoint, List<Point2D> listOfPointsInCluster)
   {
      for (int i = 0; i < listOfPointsInCluster.size(); i++)
      {
         if (i < listOfPointsInCluster.size() - 1)
         {
            //             System.out.println("i: " + i);
            Point2D first = new Point2D(listOfPointsInCluster.get(i).getX(), listOfPointsInCluster.get(i).getY());
            Point2D second = new Point2D(listOfPointsInCluster.get(i + 1).getX(), listOfPointsInCluster.get(i + 1).getY());

            //            System.out.println(first + "   " + second + "   " + observer + "   " + targetPoint);
            //            System.out.println("Intersects: " + lineSegmentsPhysicallyIntersect(first, second, observer, targetPoint));

            if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
            {
               return false;
            }
         }
      }
      return true;
   }

   public static boolean isPointInsideConcavePolygon(Point2D[] polygon, Point2D pointToCheck, Point2D lineEnd)
   {
      int index = 0;

      for (int i = 1; i < polygon.length; i++)
      {
         Point2D point1 = polygon[i - 1];
         Point2D point2 = polygon[i];

         if (EuclidGeometryTools.doLineSegment2DsIntersect(point1, point2, pointToCheck, lineEnd))
         {
            index++;
         }
      }

      //      System.out.println("INDEX: " + index);

      if (index == 0)
      {
         //Could be both outside or inside
         return false;
      }

      if (index % 2 == 0)
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   public static ArrayList<Connection> getConnectionsThatAreInsideRegion(ArrayList<Connection> connections, PlanarRegion region)
   {
      ArrayList<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (areBothPointsInside(new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY()),
                                 new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY()), region))
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

         if (areBothPointsInside(new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY()),
                                 new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY()), polygon))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }
   
   public static boolean areBothPointsInside(Point2D point1, Point2D point2, PlanarRegion homeRegion)
   {
      ArrayList<Point2D> points = new ArrayList<>();
      for (int i = 1; i < homeRegion.getConcaveHullSize(); i++)
      {
         Point2D point = homeRegion.getConcaveHull()[i];
         points.add(point);
      }

      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - point1.getX(), centroid.getY() - point1.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(point1.getX() + directionToCentroid.getX(), point1.getY() + directionToCentroid.getY());

      boolean startIsInside = VisibilityTools.isPointInsideConcavePolygon(homeRegion.getConcaveHull(), point1, endPoint);

      directionToCentroid = new Vector2D(centroid.getX() - point2.getX(), centroid.getY() - point2.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      endPoint = new Point2D(point2.getX() + directionToCentroid.getX(), point2.getY() + directionToCentroid.getY());

      boolean goalIsInside = VisibilityTools.isPointInsideConcavePolygon(homeRegion.getConcaveHull(), point2, endPoint);

      if (startIsInside && goalIsInside)
      {
         return true;
      }
      return false;
   }
   
   public static boolean areBothPointsInside(Point2D point1, Point2D point2, List<Point2D> points)
   {
      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);
      
      Vector2D directionToCentroid = new Vector2D(centroid.getX() - point1.getX(), centroid.getY() - point1.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(point1.getX() + directionToCentroid.getX(), point1.getY() + directionToCentroid.getY());

      Point2D[] pointsArr = points.toArray(new Point2D[points.size()]);
      boolean startIsInside = VisibilityTools.isPointInsideConcavePolygon(pointsArr, point1, endPoint);

      directionToCentroid = new Vector2D(centroid.getX() - point2.getX(), centroid.getY() - point2.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      endPoint = new Point2D(point2.getX() + directionToCentroid.getX(), point2.getY() + directionToCentroid.getY());

      boolean goalIsInside = VisibilityTools.isPointInsideConcavePolygon(pointsArr, point2, endPoint);

      if (startIsInside && goalIsInside)
      {
         return true;
      }
      return false;
   }

}
