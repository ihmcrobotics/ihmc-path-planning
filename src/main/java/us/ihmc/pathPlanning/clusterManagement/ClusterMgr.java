package us.ihmc.pathPlanning.clusterManagement;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.clusterManagement.Cluster.Type;

public class ClusterMgr
{
   ArrayList<Cluster> listOfClusters = new ArrayList<>();
   int extrusionIndex = 0;
   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   public void setVis(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
   }

   public void addCluster(Cluster cluster)
   {
      listOfClusters.add(cluster);
   }

   public ArrayList<Cluster> getClusters()
   {
      return listOfClusters;
   }

   public int determineExtrusionSide(Cluster cluster, Point2D observer)
   {
      int index = 0;

      for (int i = 0; i < cluster.getNormals().size(); i++)
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

   private boolean isNormalVisible(Cluster cluster, int normalIndex, Point2D observer)
   {
      for (int i = 1; i < cluster.getRawPointsInCluster().size(); i++)
      {
         Point2D target = new Point2D(cluster.getNormals().get(normalIndex).getX(), cluster.getNormals().get(normalIndex).getY());

         Point3D startPt = cluster.getRawPointsInCluster().get(i - 1);
         Point3D endPt = cluster.getRawPointsInCluster().get(i);

         Point2D startPt2d = new Point2D(startPt.getX(), startPt.getY());
         Point2D endPt2d = new Point2D(endPt.getX(), endPt.getY());

         if (EuclidGeometryTools.doLineSegment2DsIntersect(observer, target, startPt2d, endPt2d))
         {
            return false;
         }
      }
      return true;
   }

   public void generateNormalsFromRawBoundaryMap(double extrusionDistance)
   {
      for (Cluster cluster : listOfClusters)
      {
         ArrayList<Point3D> list = cluster.getRawPointsInCluster();
         for (int i = 0; i < list.size(); i++)
         {
            if (i < list.size() - 1)
            {
               Point2D first = new Point2D(list.get(i).getX(), list.get(i).getY());
               Point2D second = new Point2D(list.get(i + 1).getX(), list.get(i + 1).getY());
               generateNormalsForSegment(first, second, cluster, extrusionDistance);
               
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
   
   private void generateNormalsForSegment(Point2D first, Point2D second, Cluster cluster, double extrusionDistance)
   {
      List<Point2D> points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, 0.001);

      for (Point2D normalPoint : points)
      {
         cluster.addNormal(new Point3D(normalPoint.getX(), normalPoint.getY(), 0));
      }

      points = EuclidGeometryTools.perpendicularBisectorSegment2D(first, second, extrusionDistance + cluster.getAdditionalExtrusionDistance());

      for (Point2D normalPoint : points)
      {
         cluster.addSafeNormal(new Point3D(normalPoint.getX(), normalPoint.getY(), 0));
      }
   }

   public void performExtrusions(Point2D initialObserver, double extrusionDistance)
   {
      for (Cluster cluster : listOfClusters)
      {
         extrudeCluster(cluster, initialObserver, extrusionDistance);
      }
   }

   public void extrudeCluster(Cluster cluster, Point2D observer, double extrusionDistance)
   {

      if (cluster.getType() == Type.LINE)
      {
         //                  System.out.println("Extruding line");
         double extrusionDist1 = extrusionDistance - 0.01 + cluster.getAdditionalExtrusionDistance();
         double extrusionDist2 = extrusionDistance + cluster.getAdditionalExtrusionDistance();

         ArrayList<Point2D> nonNavExtrusions = extrudeLine(new Point2D(cluster.getRawPointsInCluster().get(0).getX(),
                                                                       cluster.getRawPointsInCluster().get(0).getY()),
                                                           new Point2D(cluster.getRawPointsInCluster().get(1).getX(),
                                                                       cluster.getRawPointsInCluster().get(1).getY()),
                                                           extrusionDist1);
         ArrayList<Point2D> navExtrusions = extrudeLine(new Point2D(cluster.getRawPointsInCluster().get(0).getX(),
                                                                    cluster.getRawPointsInCluster().get(0).getY()),
                                                        new Point2D(cluster.getRawPointsInCluster().get(1).getX(),
                                                                    cluster.getRawPointsInCluster().get(1).getY()),
                                                        extrusionDist2);

         for (Point2D pt : nonNavExtrusions)
         {
            cluster.addNonNavigableExtrusionPoint(pt);
         }

         for (Point2D pt : navExtrusions)
         {
            cluster.addNavigableExtrusionPoint(new Point3D(pt.getX(), pt.getY(), 0));
         }
      }

      if (cluster.getType() == Type.POLYGON)
      {
         //                  System.out.println("Extruding Polygon");
         generateNormalsFromRawBoundaryMap(extrusionDistance);
         
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
            extrusionIndex = determineExtrusionSide(cluster, observer);
         }

//         javaFXMultiColorMeshBuilder.addSphere(0.04f, cluster.getListOfSafeNormals().get(extrusionIndex), Color.RED);

         extrudePolygon(cluster, extrusionIndex, extrusionDistance);
      }
   }
   
   private void extrudePolygon(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      extrusionDistance = extrusionDistance + cluster.getAdditionalExtrusionDistance();

      double extrusionDist1 = extrusionDistance - 0.01;
      double extrusionDist2 = extrusionDistance;
      
//      if (cluster.isObstacleClosed())
//         extrudeFirstNonNavigable(extrusionIndex, cluster, extrusionDist1);

      extrudedNonNavigableBoundary(extrusionIndex, cluster, extrusionDist1);

//      if (cluster.isObstacleClosed())
//         extrudeLastNonNavigable(cluster, extrusionIndex, extrusionDist1);

//      if (cluster.isObstacleClosed())
//         extrudeFirstNavigable(cluster, extrusionIndex, extrusionDist1);

      extrudedNavigableBoundary(extrusionIndex, cluster, extrusionDist2);
      
//      if (cluster.isObstacleClosed())
//         extrudeLastNavigable(cluster, extrusionIndex, extrusionDist1);
   }

   private void extrudedFirstNonNavigableExtrusion(Cluster cluster, int index, double extrusionDistance)
   {
      Point3D point1 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 1);
      Point3D point2 = cluster.getRawPointsInCluster().get(0);
      Point3D point3 = cluster.getRawPointsInCluster().get(1);

      javaFXMultiColorMeshBuilder.addSphere(0.2f, point1, Color.YELLOW);
      javaFXMultiColorMeshBuilder.addSphere(0.2f, point2, Color.YELLOW);
      javaFXMultiColorMeshBuilder.addSphere(0.2f, point3, Color.YELLOW);

      Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
      Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

      Point2D normal1 = new Point2D(cluster.getListOfSafeNormals().get(cluster.getListOfSafeNormals().size() - 1).getX(),
                                    cluster.getListOfSafeNormals().get(cluster.getListOfSafeNormals().size() - 1).getY());
      Point2D normal2 = new Point2D(cluster.getListOfSafeNormals().get(0).getX(), cluster.getListOfSafeNormals().get(0).getY());

      javaFXMultiColorMeshBuilder.addSphere(0.2f, new Point3D(normal1.getX(), normal1.getY(), 0), Color.WHITE);
      javaFXMultiColorMeshBuilder.addSphere(0.2f, new Point3D(normal2.getX(), normal2.getY(), 0), Color.WHITE);

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
      cluster.addNonNavigableExtrusionPoint(adjustedIntersection);
      //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));
   }

   private void extrudedNonNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInCluster().size() - 2; i++)
      {
         Point3D point1 = cluster.getRawPointsInCluster().get(i);
         Point3D point2 = cluster.getRawPointsInCluster().get(i + 1);
         Point3D point3 = cluster.getRawPointsInCluster().get(i + 2);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

         Point2D normal1 = new Point2D(cluster.getListOfSafeNormals().get(index).getX(), cluster.getListOfSafeNormals().get(index).getY());
         Point2D normal2 = new Point2D(cluster.getListOfSafeNormals().get(index + 2).getX(), cluster.getListOfSafeNormals().get(index + 2).getY());

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
         cluster.addNonNavigableExtrusionPoint(adjustedIntersection);
         //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));

         index = index + 2;
      }

      if (cluster.isObstacleClosed())
      {
         cluster.addNonNavigableExtrusionPoint(cluster.getListOfNonNavigableExtrusions().get(0));
      }
   }

   private void extrudedNavigableBoundary(int index, Cluster cluster, double extrusionDistance)
   {
      for (int i = 0; i < cluster.getRawPointsInCluster().size() - 2; i++)
      {
         Point3D point1 = cluster.getRawPointsInCluster().get(i);
         Point3D point2 = cluster.getRawPointsInCluster().get(i + 1);
         Point3D point3 = cluster.getRawPointsInCluster().get(i + 2);

         Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
         Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

         Point2D normal1 = new Point2D(cluster.getListOfSafeNormals().get(index).getX(), cluster.getListOfSafeNormals().get(index).getY());
         Point2D normal2 = new Point2D(cluster.getListOfSafeNormals().get(index + 2).getX(), cluster.getListOfSafeNormals().get(index + 2).getY());

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

         //         cluster.addNavigableExtrusionPoint(new Point3D(safePoint1.getX(), safePoint1.getY(), 0));
         cluster.addNavigableExtrusionPoint(new Point3D(adjustedIntersection.getX(), adjustedIntersection.getY(), 0));
         //         cluster.addNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));

         index = index + 2;
      }

      if (cluster.isObstacleClosed())
      {
         cluster.addNavigableExtrusionPoint(new Point3D(cluster.getListOfNavigableExtrusions().get(0)));
      }

   }


   private void extrudeFirstNonNavigable(int index, Cluster cluster, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         for (int i = 0; i < cluster.getRawPointsInCluster().size() - 2; i++)
         {
            Point3D point1 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 1);
            Point3D point2 = cluster.getRawPointsInCluster().get(0);
            Point3D point3 = cluster.getRawPointsInCluster().get(1);

            Vector2D vec1 = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
            Vector2D vec2 = new Vector2D(point3.getX() - point2.getX(), point3.getY() - point2.getY());

            Point2D normal1 = new Point2D(cluster.getListOfSafeNormals().get(index).getX(), cluster.getListOfSafeNormals().get(index).getY());
            Point2D normal2 = new Point2D(cluster.getListOfSafeNormals().get(index + 2).getX(), cluster.getListOfSafeNormals().get(index + 2).getY());

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
            cluster.addNonNavigableExtrusionPoint(adjustedIntersection);
            //         cluster.addNonNavigableExtrusionPoint(new Point3D(safePoint2.getX(), safePoint2.getY(), 0));
         }
      }
   }

   private void extrudeLastNonNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point3D point1 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 2);
         Point3D point2 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 1);
         Point3D point3 = cluster.getRawPointsInCluster().get(0);

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
         cluster.addNonNavigableExtrusionPoint(extrudedPoint);
      }
   }

   private void extrudeFirstNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point3D point1 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 1);
         Point3D point2 = cluster.getRawPointsInCluster().get(0);
         Point3D point3 = cluster.getRawPointsInCluster().get(1);

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

         cluster.addNavigableExtrusionPoint(new Point3D(extrudedPoint.getX(), extrudedPoint.getY(), 0));
      }
   }

   private void extrudeLastNavigable(Cluster cluster, int extrusionIndex, double extrusionDistance)
   {
      if (cluster.isObstacleClosed())
      {
         //First Extrusion
         Point3D point1 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 2);
         Point3D point2 = cluster.getRawPointsInCluster().get(cluster.getRawPointsInCluster().size() - 1);
         Point3D point3 = cluster.getRawPointsInCluster().get(0);

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

         cluster.addNavigableExtrusionPoint(new Point3D(extrudedPoint.getX(), extrudedPoint.getY(), 0));
      }
   }

   private ArrayList<Point2D> extrudeLine(Point2D pt1, Point2D pt2, double extrusionDistance)
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

   private Point2D extrudeCorner(Point2D pointOnLine, Vector2D vec21, Point2D extrudedPoint1, Point2D extrudedPoint2, double extrusion)
   {
      Vector2D orthoVec = new Vector2D(vec21.getX() * Math.cos(Math.toRadians(90)) - vec21.getY() * Math.sin(Math.toRadians(90)),
                                       vec21.getX() * Math.sin(Math.toRadians(90)) + vec21.getY() * Math.cos(Math.toRadians(90)));
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
}
