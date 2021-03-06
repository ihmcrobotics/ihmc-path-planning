package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ClusterTools
{
   private static final boolean debug = false;

   public static int determineExtrusionSide(Cluster cluster, Point2D observer)
   {
      int index = 0;

      for (int i = 0; i < cluster.getNumberOfNormals(); i++)
      {
         if (isNormalVisible(cluster, i, observer))
         {
            index = i;
            break;
         }
      }

      if (index % 2 == 0)
      {
         index = 0;
      }
      else
      {
         index = 1;
      }

      return index;
   }

   private static boolean isNormalVisible(Cluster cluster, int normalIndex, Point2D observer)
   {
      List<Point2D> rawPointsInLocal = cluster.getRawPointsInLocal();
      for (int i = 1; i < rawPointsInLocal.size(); i++)
      {
         Point2D target = new Point2D(cluster.getNormalInLocal(normalIndex));

         Point2D startPt = rawPointsInLocal.get(i - 1);
         Point2D endPt = rawPointsInLocal.get(i);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(observer, target, startPt, endPt))
         {
            return false;
         }
      }
      return true;
   }

   private void extrudedFirstNonNavigableExtrusion(Cluster cluster, int index, double extrusionDistance)
   {
      Point2D point1 = cluster.getLastRawPointInLocal();
      Point2D point2 = cluster.getRawPointInLocal(0);
      Point2D point3 = cluster.getRawPointInLocal(1);

      Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
      Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

      Point2D normal1 = cluster.getLastSafeNormalInLocal();
      Point2D normal2 = cluster.getSafeNormalInLocal(0);

      Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

      if (intersectionPoint.distance(normal1) < 1E-6)
      {
         double deltaX = (normal2.getX() - normal1.getX()) / 2.0;
         double deltaY = (normal2.getY() - normal1.getY()) / 2.0;

         intersectionPoint.setX(normal1.getX() + deltaX);
         intersectionPoint.setY(normal2.getY() + deltaY);
      }

      Vector2D normalIntersection = new Vector2D(intersectionPoint.getX() - point2.getX(), intersectionPoint.getY() - point2.getY());
      normalIntersection.normalize();

      Point2D adjustedIntersection = new Point2D(point2.getX() + normalIntersection.getX() * (extrusionDistance),
                                                 point2.getY() + normalIntersection.getY() * (extrusionDistance));

      double x1 = point1.getX() + ((point2.getX() - point1.getX()) * 0.5);
      double y1 = point1.getY() + ((point2.getY() - point1.getY()) * 0.5);
      Point2D midPoint1 = new Point2D(x1, y1);

      double x2 = point2.getX() + ((point3.getX() - point2.getX()) * 0.5);
      double y2 = point2.getY() + ((point3.getY() - point2.getY()) * 0.5);
      Point2D midPoint2 = new Point2D(x2, y2);

      Vector2D vec21 = new Vector2D(normal1.getX() - midPoint1.getX(), normal1.getY() - midPoint1.getY());
      Point2D safePoint1 = new Point2D(point2.getX() + vec21.getX() * 0.7, point2.getY() + vec21.getY() * 0.7);

      Vector2D vec32 = new Vector2D(normal2.getX() - midPoint2.getX(), normal2.getY() - midPoint2.getY());
      Point2D safePoint2 = new Point2D(point2.getX() + vec32.getX() * 0.7, point2.getY() + vec32.getY() * 0.7);

      //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint1.getX(), safePoint1.getY(), 0));
      cluster.addNonNavigableExtrusionInLocal(adjustedIntersection);
      //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));
   }

   public static void extrudedNonNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInLocal().size() - 2; i++)
      {
         Point2D point1 = cluster.getRawPointInLocal(i);
         Point2D point2 = cluster.getRawPointInLocal(i + 1);
         Point2D point3 = cluster.getRawPointInLocal(i + 2);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

         Point2D normal1 = cluster.getSafeNormalInLocal(index);
         Point2D normal2 = cluster.getSafeNormalInLocal(index + 2);

         Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

         if (intersectionPoint == null)
         {
            if(debug)
               PrintTools.error("Failed to extrude non-navigable boundary for region " + index + " \n" + "point1: " + point1 + "\n" + "point2: " + point2 + "\n"
                     + "point3: " + point3 + "\n" + "vec1: " + vec1 + "\n" + "vec2: " + vec2 + "\n" + "normal1: " + normal1 + "\n" + "normal2: " + normal2 + "\n"
                     + "extrusionDistance: " + extrusionDistance);
            continue;
         }

         if (intersectionPoint.distance(normal1) < 1E-6)
         {
            double deltaX = (normal2.getX() - normal1.getX()) / 2.0;
            double deltaY = (normal2.getY() - normal1.getY()) / 2.0;

            intersectionPoint.setX(normal1.getX() + deltaX);
            intersectionPoint.setY(normal2.getY() + deltaY);
         }

         Vector2D directionOfIntersectionExtrusion = new Vector2D(intersectionPoint.getX() - point2.getX(), intersectionPoint.getY() - point2.getY());
         directionOfIntersectionExtrusion.normalize();

         Point2D adjustedIntersection = new Point2D(point2.getX() + directionOfIntersectionExtrusion.getX() * (extrusionDistance),
                                                    point2.getY() + directionOfIntersectionExtrusion.getY() * (extrusionDistance));

         double x1 = point1.getX() + ((point2.getX() - point1.getX()) * 0.5);
         double y1 = point1.getY() + ((point2.getY() - point1.getY()) * 0.5);
         Point2D midPoint1 = new Point2D(x1, y1);

         double x2 = point2.getX() + ((point3.getX() - point2.getX()) * 0.5);
         double y2 = point2.getY() + ((point3.getY() - point2.getY()) * 0.5);
         Point2D midPoint2 = new Point2D(x2, y2);

         Vector2D vec21 = new Vector2D(normal1.getX() - midPoint1.getX(), normal1.getY() - midPoint1.getY());
         Point2D safePoint1 = new Point2D(point2.getX() + vec21.getX() * 0.7, point2.getY() + vec21.getY() * 0.7);

         Vector2D vec32 = new Vector2D(normal2.getX() - midPoint2.getX(), normal2.getY() - midPoint2.getY());
         Point2D safePoint2 = new Point2D(point2.getX() + vec32.getX() * 0.7, point2.getY() + vec32.getY() * 0.7);

         //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint1.getX(), safePoint1.getY(), 0));
         cluster.addNonNavigableExtrusionInLocal(adjustedIntersection);
         //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));

         index = index + 2;
      }

      if (cluster.isObstacleClosed() && !cluster.getNonNavigableExtrusionsInLocal().isEmpty())
      {
         cluster.addNonNavigableExtrusionInLocal(cluster.getNonNavigableExtrusionsInLocal().get(0));
      }
   }

   public static void extrudedNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInLocal().size() - 2; i++)
      {
         Point2D point1 = cluster.getRawPointInLocal(i);
         Point2D point2 = cluster.getRawPointInLocal(i + 1);
         Point2D point3 = cluster.getRawPointInLocal(i + 2);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

         Point2D normal1 = cluster.getSafeNormalInLocal(index);
         Point2D normal2 = cluster.getSafeNormalInLocal(index + 2);

         Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

         if (intersectionPoint == null)
         {
            if(debug)
               PrintTools.error("Failed to extrude navigable boundary for region " + index + " \n" + "point1: " + point1 + "\n" + "point2: " + point2 + "\n"
                     + "point3: " + point3 + "\n" + "vec1: " + vec1 + "\n" + "vec2: " + vec2 + "\n" + "normal1: " + normal1 + "\n" + "normal2: " + normal2 + "\n"
                     + "extrusionDistance: " + extrusionDistance);
            continue;
         }

         if (intersectionPoint.distance(normal1) < 1E-6)
         {
            double deltaX = (normal2.getX() - normal1.getX()) / 2.0;
            double deltaY = (normal2.getY() - normal1.getY()) / 2.0;

            intersectionPoint.setX(normal1.getX() + deltaX);
            intersectionPoint.setY(normal2.getY() + deltaY);
         }

         Vector2D normalIntersection = new Vector2D(intersectionPoint.getX() - point2.getX(), intersectionPoint.getY() - point2.getY());
         normalIntersection.normalize();

         Point2D adjustedIntersection = new Point2D(point2.getX() + normalIntersection.getX() * (extrusionDistance),
                                                    point2.getY() + normalIntersection.getY() * (extrusionDistance));

         double x1 = point1.getX() + ((point2.getX() - point1.getX()) * 0.5);
         double y1 = point1.getY() + ((point2.getY() - point1.getY()) * 0.5);
         Point2D midPoint1 = new Point2D(x1, y1);

         double x2 = point2.getX() + ((point3.getX() - point2.getX()) * 0.5);
         double y2 = point2.getY() + ((point3.getY() - point2.getY()) * 0.5);
         Point2D midPoint2 = new Point2D(x2, y2);

         Vector2D vec21 = new Vector2D(normal1.getX() - midPoint1.getX(), normal1.getY() - midPoint1.getY());
         Point2D safePoint1 = new Point2D(point2.getX() + vec21.getX() * 0.7, point2.getY() + vec21.getY() * 0.7);

         Vector2D vec32 = new Vector2D(normal2.getX() - midPoint2.getX(), normal2.getY() - midPoint2.getY());
         Point2D safePoint2 = new Point2D(point2.getX() + vec32.getX() * 0.7, point2.getY() + vec32.getY() * 0.7);

         //         cluster.addNavigableExtrusionPoint(new Point3D(safePoint1.getX(), safePoint1.getY(), 0));
         cluster.addNavigableExtrusionInLocal(adjustedIntersection);
         //         cluster.addNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));

         index = index + 2;
      }

      if (cluster.isObstacleClosed() && !cluster.getNavigableExtrusionsInLocal().isEmpty())
      {
         cluster.addNavigableExtrusionInLocal(cluster.getNavigableExtrusionInLocal(0));
      }

   }

   private void extrudeFirstNonNavigable(int index, Cluster cluster, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         for (int i = 0; i < cluster.getRawPointsInLocal().size() - 2; i++)
         {
            Point2D point1 = cluster.getLastRawPointInLocal();
            Point2D point2 = cluster.getRawPointInLocal(0);
            Point2D point3 = cluster.getRawPointInLocal(1);

            Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
            Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

            Point2D normal1 = cluster.getSafeNormalInLocal(index);
            Point2D normal2 = cluster.getSafeNormalInLocal(index + 2);

            Point2D intersectionPoint = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(normal1, vec1, normal2, vec2);

            if (intersectionPoint.distance(normal1) < 1E-6)
            {
               double deltaX = (normal2.getX() - normal1.getX()) / 2.0;
               double deltaY = (normal2.getY() - normal1.getY()) / 2.0;

               intersectionPoint.setX(normal1.getX() + deltaX);
               intersectionPoint.setY(normal2.getY() + deltaY);
            }

            Vector2D normalIntersection = new Vector2D(intersectionPoint.getX() - point2.getX(), intersectionPoint.getY() - point2.getY());
            normalIntersection.normalize();

            Point2D adjustedIntersection = new Point2D(point2.getX() + normalIntersection.getX() * (extrusionDistance),
                                                       point2.getY() + normalIntersection.getY() * (extrusionDistance));

            double x1 = point1.getX() + ((point2.getX() - point1.getX()) * 0.5);
            double y1 = point1.getY() + ((point2.getY() - point1.getY()) * 0.5);
            Point2D midPoint1 = new Point2D(x1, y1);

            double x2 = point2.getX() + ((point3.getX() - point2.getX()) * 0.5);
            double y2 = point2.getY() + ((point3.getY() - point2.getY()) * 0.5);
            Point2D midPoint2 = new Point2D(x2, y2);

            Vector2D vec21 = new Vector2D(normal1.getX() - midPoint1.getX(), normal1.getY() - midPoint1.getY());
            Point2D safePoint1 = new Point2D(point2.getX() + vec21.getX() * 0.7, point2.getY() + vec21.getY() * 0.7);

            Vector2D vec32 = new Vector2D(normal2.getX() - midPoint2.getX(), normal2.getY() - midPoint2.getY());
            Point2D safePoint2 = new Point2D(point2.getX() + vec32.getX() * 0.7, point2.getY() + vec32.getY() * 0.7);

            //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint1.getX(), safePoint1.getY(), 0));
            cluster.addNonNavigableExtrusionInLocal(adjustedIntersection);
            //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));
         }
      }
   }

   private void extrudeLastNonNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point2D point1 = cluster.getRawPointInLocal(cluster.getRawPointsInLocal().size() - 2);
         Point2D point2 = cluster.getRawPointInLocal(cluster.getRawPointsInLocal().size() - 1);
         Point2D point3 = cluster.getRawPointInLocal(0);

         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point1, Color.RED);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point2, Color.RED);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point3, Color.RED);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point2.getX() - point3.getX(), point2.getY() - point3.getY());
         vec1.normalize();
         vec2.normalize();

         Vector2D dirVector = new Vector2D(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY());
         dirVector.normalize();

         Point2D extrudedPoint = new Point2D(point2.getX() + dirVector.getX() * extrusionDistance, point2.getY() + dirVector.getY() * extrusionDistance);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, new Point3D(extrudedPoint.getX(), extrudedPoint.getY(), 0), Color.YELLOW);
         //
         cluster.addNonNavigableExtrusionInLocal(extrudedPoint);
      }
   }

   private void extrudeFirstNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point2D point1 = cluster.getRawPointInLocal(cluster.getRawPointsInLocal().size() - 1);
         Point2D point2 = cluster.getRawPointInLocal(0);
         Point2D point3 = cluster.getRawPointInLocal(1);

         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point1, Color.YELLOW);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point2, Color.YELLOW);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point3, Color.YELLOW);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point2.getX() - point3.getX(), point2.getY() - point3.getY());
         vec1.normalize();
         vec2.normalize();

         Vector2D dirVector = new Vector2D(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY());
         dirVector.normalize();

         Point2D extrudedPoint = new Point2D(point2.getX() + dirVector.getX() * extrusionDistance, point2.getY() + dirVector.getY() * extrusionDistance);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, new Point3D(extrudedPoint.getX(), extrudedPoint.getY(), 0), Color.YELLOW);

         cluster.addNavigableExtrusionInLocal(extrudedPoint);
      }
   }

   private void extrudeLastNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point2D point1 = cluster.getRawPointInLocal(cluster.getRawPointsInLocal().size() - 2);
         Point2D point2 = cluster.getRawPointInLocal(cluster.getRawPointsInLocal().size() - 1);
         Point2D point3 = cluster.getRawPointInLocal(0);

         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point1, Color.YELLOW);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point2, Color.YELLOW);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, point3, Color.YELLOW);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point2.getX() - point3.getX(), point2.getY() - point3.getY());
         vec1.normalize();
         vec2.normalize();

         Vector2D dirVector = new Vector2D(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY());
         dirVector.normalize();

         Point2D extrudedPoint = new Point2D(point2.getX() + dirVector.getX() * extrusionDistance, point2.getY() + dirVector.getY() * extrusionDistance);
         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, new Point3D(extrudedPoint.getX(), extrudedPoint.getY(), 0), Color.YELLOW);

         cluster.addNavigableExtrusionInLocal(extrudedPoint);
      }
   }

   public static ArrayList<Point2D> extrudeLine(Point2D pt1, Point2D pt2, double extrusionDistance)
   {
      ArrayList<Point2D> points = new ArrayList<>();

      Vector2D vec21 = new Vector2D(pt2.getX() - pt1.getX(), pt2.getY() - pt1.getY());

      vec21.normalize();

      Point2D endExtrusion1 = new Point2D(pt2.getX() + vec21.getX() * extrusionDistance, pt2.getY() + vec21.getY() * extrusionDistance);
      vec21.negate();
      Point2D endExtrusion2 = new Point2D(pt1.getX() + vec21.getX() * extrusionDistance, pt1.getY() + vec21.getY() * extrusionDistance);

      Point2D midNormal1 = EuclidGeometryTools.perpendicularBisectorSegment2D(pt1, pt2, extrusionDistance).get(0);
      Point2D midNormal2 = EuclidGeometryTools.perpendicularBisectorSegment2D(pt1, pt2, extrusionDistance).get(1);

      points.add(endExtrusion2);
      points.add(extrudeCorner(pt1, vec21, endExtrusion2, midNormal1, extrusionDistance));
      points.add(midNormal1);
      points.add(extrudeCorner(pt2, vec21, endExtrusion1, midNormal1, extrusionDistance));
      points.add(endExtrusion1);
      points.add(extrudeCorner(pt2, vec21, endExtrusion1, midNormal2, extrusionDistance));
      points.add(midNormal2);
      points.add(extrudeCorner(pt1, vec21, endExtrusion2, midNormal2, extrusionDistance));
      points.add(endExtrusion2);

      return points;
   }

   // TODO That method isn't very clear
   private static Point2D extrudeCorner(Point2D pointOnLine, Vector2D vec21, Point2D extrudedPoint1, Point2D extrudedPoint2, double extrusion)
   {
      Vector2D orthoVec = new Vector2D(vec21.getX() * Math.cos(Math.toRadians(90)) - vec21.getY() * Math.sin(Math.toRadians(90)),
                                       vec21.getX() * Math.sin(Math.toRadians(90)) + vec21.getY() * Math.cos(Math.toRadians(90)));
      // TODO isn't it the same as:
      //      Vector2D orthoVec = EuclidGeometryTools.perpendicularVector2D(vec21);

      Point2D inter1 = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(extrudedPoint1, orthoVec, extrudedPoint2, vec21);

      Vector2D vecExtrToCorner = new Vector2D(inter1.getX() - pointOnLine.getX(), inter1.getY() - pointOnLine.getY());
      vecExtrToCorner.normalize();

      Point2D extr1 = new Point2D(pointOnLine.getX() + vecExtrToCorner.getX() * extrusion, pointOnLine.getY() + vecExtrToCorner.getY() * extrusion);

      //      VisualizationTool.visualizePoint(new Point3D(extr1.x, extr1.y, height), ColorRGBA.Red, 0.065f);

      //      DebugSphere top1 = new DebugSphere(basicJmeApp, 0.065f, 10, 10, ColorRGBA.Red);
      //      basicJmeApp.zUpNode.attachChild(top1);
      //      top1.setLocalTranslation((float) extr1.x, (float) extr1.y, height);

      return extr1;
   }

   public static void extrudeCluster(Cluster cluster, Point2D observer, double extrusionDistance, List<Cluster> listOfClusters)
   {
      int extrusionIndex = 0;
      if (cluster.getType() == Type.LINE)
      {
         //         System.out.println("Extruding line");
         //         System.out.println("Distance: " + extrusionDistance);

         double extrusionDist1 = extrusionDistance - 0.01 + cluster.getAdditionalExtrusionDistance();
         double extrusionDist2 = extrusionDistance + cluster.getAdditionalExtrusionDistance();

         //         System.out.println(extrusionDist1 + "   " + extrusionDist2);

         ArrayList<Point2D> nonNavExtrusions = ClusterTools.extrudeLine(new Point2D(cluster.getRawPointsInLocal().get(0).getX(),
                                                                                    cluster.getRawPointsInLocal().get(0).getY()),
                                                                        new Point2D(cluster.getRawPointsInLocal().get(1).getX(),
                                                                                    cluster.getRawPointsInLocal().get(1).getY()),
                                                                        extrusionDist1);
         ArrayList<Point2D> navExtrusions = ClusterTools.extrudeLine(new Point2D(cluster.getRawPointsInLocal().get(0).getX(),
                                                                                 cluster.getRawPointsInLocal().get(0).getY()),
                                                                     new Point2D(cluster.getRawPointsInLocal().get(1).getX(),
                                                                                 cluster.getRawPointsInLocal().get(1).getY()),
                                                                     extrusionDist2);

         for (Point2D pt : nonNavExtrusions)
         {
            cluster.addNonNavigableExtrusionInLocal(pt);
         }

         for (Point2D pt : navExtrusions)
         {
            cluster.addNavigableExtrusionInLocal(pt);
         }
      }

      if (cluster.getType() == Type.POLYGON)
      {
         //                  System.out.println("Extruding Polygon");
         generateNormalsFromRawBoundaryMap(extrusionDistance, listOfClusters);

         if (cluster.isObstacleClosed() && cluster.getExtrusionSide() != ExtrusionSide.AUTO)
         {
            if (cluster.getExtrusionSide() == ExtrusionSide.INSIDE)
            {
               extrusionIndex = 1;
            }
            else
            {
               extrusionIndex = 0;
            }
         }
         else
         {
            extrusionIndex = ClusterTools.determineExtrusionSide(cluster, observer);
         }

         //         javaFXMultiColorMeshBuilder.addSphere(0.04f, cluster.getListOfSafeNormals().get(extrusionIndex), Color.RED);

         extrudePolygon(cluster, extrusionIndex, extrusionDistance);
      }
   }

   public static void extrudePolygon(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      extrusionDistance = extrusionDistance + cluster.getAdditionalExtrusionDistance();

      double extrusionDist1 = extrusionDistance - 0.01;
      double extrusionDist2 = extrusionDistance;

      //      if (cluster.isObstacleClosed())
      //         extrudeFirstNonNavigable(extrusionIndex, cluster, extrusionDist1);

      ClusterTools.extrudedNonNavigableBoundary(extrusionIndex, cluster, extrusionDist1);

      //      if (cluster.isObstacleClosed())
      //         extrudeLastNonNavigable(cluster, extrusionIndex, extrusionDist1);

      //      if (cluster.isObstacleClosed())
      //         extrudeFirstNavigable(cluster, extrusionIndex, extrusionDist1);

      ClusterTools.extrudedNavigableBoundary(extrusionIndex, cluster, extrusionDist2);

      //      if (cluster.isObstacleClosed())
      //         extrudeLastNavigable(cluster, extrusionIndex, extrusionDist1);
   }
   
   public static void generateNormalsFromRawBoundaryMap(double extrusionDistance, List<Cluster> listOfClusters)
   {
      for (Cluster cluster : listOfClusters)
      {
         List<Point2D> rawPoints = cluster.getRawPointsInLocal();
         for (int i = 0; i < rawPoints.size(); i++)
         {
            if (i < rawPoints.size() - 1)
            {
               Point2D first = rawPoints.get(i);
               Point2D second = rawPoints.get(i + 1);
               generateNormalsForSegment(first, second, cluster, extrusionDistance);

               // TODO Remove following?
               //               if(cluster.isObstacleClosed())
               //               {
               ////                  first = new Point2D(list.get(list.size() - 1).getX(), list.get(list.size() - 1).getY());
               ////                  second = new Point2D(list.get(0).getX(), list.get(0).getY());
               ////                  generateNormalsForSegment(first, second, cluster, extrusionDistance);
               //
               ////                  first = new Point2D(list.get(0).getX(), list.get(0).getY());
               ////                  second = new Point2D(list.get(1).getX(), list.get(1).getY());
               ////                  generateNormalsForSegment(first, second, cluster, extrusionDistance);
               //               }
            }
         }
      }
   }

   public static void generateNormalsForSegment(Point2DReadOnly first, Point2DReadOnly second, Cluster cluster, double extrusionDistance)
   {
      List<Point2D> points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, 0.001);

      for (Point2D normalPoint : points)
      {
         cluster.addNormalInLocal(normalPoint);
      }

      points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, extrusionDistance + cluster.getAdditionalExtrusionDistance());

      for (Point2D normalPoint : points)
      {
         cluster.addSafeNormalInLocal(normalPoint);
      }
   }

   
   public static void classifyExtrusions(List<PlanarRegion> regionsToProject, PlanarRegion regionToProjectTo, List<PlanarRegion> lineObstaclesToPack, List<PlanarRegion> polygonObstaclesToPack, double zNormalThreshold)
   {
      
      for(PlanarRegion regionToProject : regionsToProject)
      {
         Vector3D normal = PlanarRegionTools.calculateNormal(regionToProject);

         if (normal != null && regionToProject != regionToProjectTo)
         {
            //         System.out.println(Math.abs(normal.getZ()) + "   " + VisibilityGraphsParameters.NORMAL_Z_THRESHOLD_FOR_POLYGON_OBSTACLES);

            if (Math.abs(normal.getZ()) < zNormalThreshold)
            {
               //            System.out.println("Adding a line obstacle");
               lineObstaclesToPack.add(regionToProject);
            }
            else
            {
               //            System.out.println("Adding a polygon obstacle");
               polygonObstaclesToPack.add(regionToProject);
            }
         }

         //            System.out.println("Total obstacles to classify: " + regionsInsideHomeRegion.size() + "  Line obstacles: " + lineObstacleRegions.size()
         //                  + "   Polygon obstacles: " + polygonObstacleRegions.size());
      }
   }
   
   public static void createClusterForHomeRegion(List<Cluster> clusters, RigidBodyTransform transformToWorld, PlanarRegion homeRegion, double extrusionDistance)
   {
      Cluster cluster = new Cluster();
      clusters.add(cluster);
      cluster.setType(Type.POLYGON);
      cluster.setTransformToWorld(transformToWorld);

      Point2D[] concaveHull = homeRegion.getConcaveHull();
      for (Point2D vertex : concaveHull)
      {
         cluster.addRawPointInLocal(vertex);
         //         javaFXMultiColorMeshBuilder.addSphere(0.05f, new Point3D(pointToProject.getX(), pointToProject.getY(), pointToProject.getZ()), Color.GREEN);
      }

      cluster.setClusterClosure(true);
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);
      cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.1));
   }
   
   public static void createClustersFromRegions(PlanarRegion homeRegion, List<PlanarRegion> regions, List<PlanarRegion> lineObstacleRegions, List<PlanarRegion> polygonObstacleRegions, List<Cluster> clusters, RigidBodyTransform transformToWorld, VisibilityGraphsParameters visibilityGraphsParameters)
   {
      for (PlanarRegion region : lineObstacleRegions)
      {
         if (regions.contains(region))
         {
            //                        System.out.println("Creating a line cluster");
            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.LINE);
            cluster.setTransformToWorld(transformToWorld);

            if (PlanarRegionTools.isRegionTooHighToStep(region, homeRegion, visibilityGraphsParameters.getTooHighToStepDistance()))
            {
               cluster.setAdditionalExtrusionDistance(0);
            }
            else
            {
               cluster.setAdditionalExtrusionDistance(visibilityGraphsParameters.getExtrusionDistanceIfNotTooHighToStep()
                     - visibilityGraphsParameters.getExtrusionDistance());

               //               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance - 0.01));
               //               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance * 0.6));
            }

            Vector3D normal = PlanarRegionTools.calculateNormal(homeRegion);
            ArrayList<Point3D> points = new ArrayList<>();
            for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
            {
               Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
               Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
               FramePoint3D fpt = new FramePoint3D();
               fpt.set(point3D);
               RigidBodyTransform transToWorld = new RigidBodyTransform();
               region.getTransformToWorld(transToWorld);
               fpt.applyTransform(transToWorld);

               Point3D pointToProject = fpt.getPoint();
               Point3D projectedPoint = new Point3D();
               EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);
               points.add(projectedPoint);
            }

            LinearRegression3D linearRegression = new LinearRegression3D(points);
            linearRegression.calculateRegression();

            //Convert to local frame
            Point3D[] extremes = linearRegression.getTheTwoPointsFurthestApart();
            FramePoint3D extreme1Fpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), extremes[0]);
            FramePoint3D extreme2Fpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), extremes[1]);

            cluster.addRawPointInWorld(extreme1Fpt.getPoint());
            cluster.addRawPointInWorld(extreme2Fpt.getPoint());

            //                           javaFXMultiColorMeshBuilder.addLine(extreme1Fpt.getPoint(), extreme2Fpt.getPoint(), 0.005, Color.BLUE);
         }
      }

      //      System.out.println(polygonObstacleRegions.size());

      for (PlanarRegion region : polygonObstacleRegions)
      {
         if (regions.contains(region))
         {
            //            System.out.println("Creating a polygon cluster");

            Cluster cluster = new Cluster();
            clusters.add(cluster);
            cluster.setType(Type.POLYGON);
            cluster.setTransformToWorld(transformToWorld);

            Vector3D normal1 = PlanarRegionTools.calculateNormal(region);
            if (Math.abs(normal1.getZ()) >= 0.5) //if its closer to being flat you can probably step on it -->> extrude less
            {
               cluster.setAdditionalExtrusionDistance(visibilityGraphsParameters.getExtrusionDistanceIfNotTooHighToStep()
                     - visibilityGraphsParameters.getExtrusionDistance());
               //               cluster.setAdditionalExtrusionDistance(-1.0 * (extrusionDistance * 0.7));

               if (PlanarRegionTools.isRegionTooHighToStep(region, homeRegion, visibilityGraphsParameters.getTooHighToStepDistance())) //is flat but too high to step so its an obstacle
               {
                  cluster.setAdditionalExtrusionDistance(0);
               }
               else
               {

               }
            }

            Vector3D normal = PlanarRegionTools.calculateNormal(homeRegion);
            for (int i = 0; i < region.getConcaveHullSize(); i++)
            {
               Point2D point2D = (Point2D) region.getConcaveHull()[i];
               Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
               FramePoint3D fpt = new FramePoint3D();
               fpt.set(point3D);
               RigidBodyTransform transToWorld = new RigidBodyTransform();
               region.getTransformToWorld(transToWorld);
               fpt.applyTransform(transToWorld);

               Point3D pointToProject = fpt.getPoint();
               Point3D projectedPoint = new Point3D();
               EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPoint);

               FramePoint3D pointFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), projectedPoint);

               //               System.out.println(pointFpt);

               cluster.addRawPointInWorld(pointFpt.getPoint());
            }

            cluster.setClusterClosure(true);
         }
      }

      if (debug)
      {
         for (Cluster cluster : clusters)
         {
            System.out.println("Created a cluster of type: " + cluster.getType() + " with " + cluster.getRawPointsInLocal().size() + " points");
         }
      }
   }
   
   public static void performExtrusions(Point2D initialObserver, double extrusionDistance, List<Cluster> clusters)
   {
      for (Cluster cluster : clusters)
      {
         ClusterTools.extrudeCluster(cluster, initialObserver, extrusionDistance, clusters);
      }
   }

}
