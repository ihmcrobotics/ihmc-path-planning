package us.ihmc.pathPlanning.visibilityGraphs.tools;
import java.util.ArrayList;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2D observer, Point2D targetPoint, ArrayList<Point2D> listOfPointsInCluster)
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

   public static boolean lineSegmentsPhysicallyIntersect(Point2D line1StartPoint, Point2D line1EndPoint, Point2D line2StartPoint, Point2D line2EndPoint)
   {
      if ((line2StartPoint.distance(line1StartPoint) < 0.001) || (line2StartPoint.distance(line1EndPoint) < 0.001)
            || (line2EndPoint.distance(line1StartPoint) < 0.001) || (line2EndPoint.distance(line1EndPoint) < 0.001))
      {
         System.out.println("State 1");
         //         System.out.println(line2StartPoint.distance(line1StartPoint) + "   " + line2StartPoint.distance(line1EndPoint) + "   " + line2EndPoint.distance(line1StartPoint) + "   " + (line2EndPoint.distance(line1EndPoint)));
         return true;
      }

      //Completely on the left side
      else if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line2StartPoint, line1StartPoint, line1EndPoint)
            && EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line2EndPoint, line1StartPoint, line1EndPoint))
      {
         System.out.println("State 2");

         return false;
      }

      //Completely on the right side
      else if (!EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line2StartPoint, line1StartPoint, line1EndPoint)
            && !EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line2EndPoint, line1StartPoint, line1EndPoint))
      {
         System.out.println("State 3");

         return false;
      }

//      //Completely on the left side
//      else if (EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line1StartPoint, line2StartPoint, line2EndPoint)
//            && EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line1EndPoint, line2StartPoint, line2EndPoint))
//      {
//         System.out.println("3");
//         return false;
//      }
//
//      //Completely on the right side
//      else if (!EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line1StartPoint, line2StartPoint, line2EndPoint)
//            && !EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(line1EndPoint, line2StartPoint, line2EndPoint))
//      {
//         System.out.println("4");
//         return false;
//      }
      else
      {
         return true;
      }
   }
}
