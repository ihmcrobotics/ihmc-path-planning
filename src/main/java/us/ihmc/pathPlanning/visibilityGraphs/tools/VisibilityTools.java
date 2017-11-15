package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;

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
}
