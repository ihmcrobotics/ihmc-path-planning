package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class PointCloudTools
{
   public enum WindingOrder
   {
      CW, CCW
   };

   private static double getAngle(Point3D pt1, Point3D refPt)
   {
      return Math.atan2(pt1.getY() - refPt.getY(), pt1.getX() - refPt.getX());
   }

   public static ArrayList<Point3D> orderPoints(ArrayList<Point3D> pointsToBeOrdered, WindingOrder windingOrder, Point3D referencePoint)
   {
      ArrayList<PointOrderHolder> listOfPointHolders = new ArrayList<>();
      for (Point3D pt : pointsToBeOrdered)
      {
         listOfPointHolders.add(new PointOrderHolder(new Point2D(pt.getX(), pt.getY()), getAngle(pt, referencePoint)));
      }

      ArrayList<PointOrderHolder> orderedPointHolders = new ArrayList<>();

      if (windingOrder == WindingOrder.CW)
      {
         while (!listOfPointHolders.isEmpty())
         {
            double maxAngle = -190.0;
            PointOrderHolder maxPointHolder = null;
            for (PointOrderHolder pointHolder : listOfPointHolders)
            {
               if (pointHolder.angle > maxAngle)
               {
                  maxAngle = pointHolder.angle;
                  maxPointHolder = pointHolder;
               }
            }

            listOfPointHolders.remove(maxPointHolder);
            //            System.out.println("Adding " + Math.toDegrees(maxPointHolder.angle));
            orderedPointHolders.add(maxPointHolder);
         }
      }
      else
      {
         while (!listOfPointHolders.isEmpty())
         {
            double minAngle = 190.0;
            PointOrderHolder maxPointHolder = null;
            for (PointOrderHolder pointHolder : listOfPointHolders)
            {
               if (pointHolder.angle < minAngle)
               {
                  minAngle = pointHolder.angle;
                  maxPointHolder = pointHolder;
               }
            }

            listOfPointHolders.remove(maxPointHolder);
            //            System.out.println("Adding " + Math.toDegrees(maxPointHolder.angle));
            orderedPointHolders.add(maxPointHolder);
         }
      }

      for (int i = 0; i < orderedPointHolders.size() - 1; i++)
      {
         Point2D ptA = orderedPointHolders.get(i).point;
         Point2D ptB = orderedPointHolders.get(i + 1).point;

         isClockwise(new Point2D(ptA.getX(), ptA.getY()), new Point2D(ptB.getX(), ptB.getY()), new Point2D(referencePoint.getX(), referencePoint.getY()));
      }

      ArrayList<Point3D> orderedList = new ArrayList<>();
      for (PointOrderHolder pointOrderHolder : orderedPointHolders)
      {
         orderedList.add(new Point3D((float)pointOrderHolder.point.getX(), (float)pointOrderHolder.point.getY(), 0));
      }

      return orderedList;
   }
   
   private static class PointOrderHolder
   {
      private Point2D point;
      private double angle;

      public PointOrderHolder(Point2D point, double angle)
      {
         this.point = point;
         this.angle = angle;
      }
   }
   
   public static Point3D getCentroid(ArrayList<Point3D> listOfPoints)
   {
      double x = 0;
      double y = 0;
      double z = 0;
      for (int i = 0; i < listOfPoints.size(); i++)
      {
         x = x + listOfPoints.get(i).getX();
         y = y + listOfPoints.get(i).getY();
         z = z + listOfPoints.get(i).getZ();
      }
      Point3D centroid1 = new Point3D((x / listOfPoints.size()),(y / listOfPoints.size()),(z / listOfPoints.size()));

      return centroid1;
   }

   private static boolean isClockwise(Point2D ptA, Point2D ptB, Point2D refPt)
   {
      if (calculateDeterminant(ptA, ptB, refPt) > 0)
      {
         //         System.out.println("CCW");
         return false;
      }
      if (calculateDeterminant(ptA, ptB, refPt) < 0)
      {
         //         System.out.println("CW");
         return true;
      }

      return true;
   }

   private static double calculateDeterminant(Point2D ptA, Point2D ptB, Point2D refPt)
   {
      return (ptA.getX() - refPt.getX()) * (ptB.getY() - refPt.getY()) - (ptB.getX() - refPt.getX()) * (ptA.getY() - refPt.getY());
   }

   public static void main(String args[])
   {
      new PointCloudTools();

      //      Point3f pt1 = new Point3f();
      //      Point3f refPt = new Point3f();
      //      
      //      pt1 = new Point3f(1,1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //
      //      pt1 = new Point3f(1,0,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(1,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(0,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(-1,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(-1,0,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(0,1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
   }
}
