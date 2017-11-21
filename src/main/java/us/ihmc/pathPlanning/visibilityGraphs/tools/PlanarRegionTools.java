package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionTools
{
   private static final boolean debug = false;

   /**
    * Finds and returns the closest point the the provided point on the planar regions.
    */
   public static Point3D projectPointToPlanes(Point3DReadOnly point, PlanarRegionsList regions)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestPoint = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = closestPointOnPlane(point, region);
         double distance = intersection.distance(point);

         if (closestPoint == null || distance < smallestDistance)
         {
            smallestDistance = distance;
            closestPoint = intersection;
         }
      }

      return closestPoint;
   }

   /**
    * Projects the given point onto the planar region, returning the closest point on
    * the region to the provided point.
    */
   public static Point3D closestPointOnPlane(Point3DReadOnly point, PlanarRegion region)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, point, planeNormal);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      Point2D intersectionInPlaneFrame2D = new Point2D(intersectionInPlaneFrame);

      // checking convex hull here - might be better to check all polygons to avoid false positive
      ConvexPolygon2D convexHull = region.getConvexHull();
      if (!convexHull.isPointInside(intersectionInPlaneFrame2D))
      {
         convexHull.orthogonalProjection(intersectionInPlaneFrame2D);
         intersectionWithPlane.setToZero();
         intersectionWithPlane.set(intersectionInPlaneFrame2D);
         intersectionWithPlane.applyTransform(regionToWorld);
      }
      return intersectionWithPlane;
   }

   /**
    * Projects the given point onto a planar region from the list. The projection is done along
    * the z axis in world frame and if there is multiple regions that the point can be
    * projected onto, the highest intersection point will be returned.
    * <p>
    * Will return null if the is no planar region above or below the point.
    */
   public static Point3D projectPointToPlanesVertically(Point3DReadOnly point, PlanarRegionsList regions)
   {
      Line3D projectionLine = new Line3D(point, new Vector3D(0.0, 0.0, 1.0));
      Point3D highestIntersection = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = intersectRegionWithLine(region, projectionLine);

         if (intersection == null)
         {
            continue;
         }

         if (highestIntersection == null || highestIntersection.getZ() < intersection.getZ())
         {
            highestIntersection = intersection;
         }
      }

      return highestIntersection;
   }

   /**
    * Will return the intersection point between a line and a single planar region. If
    * the line does not intersect the region this method will return null.
    */
   public static Point3D intersectRegionWithLine(PlanarRegion region, Line3D projectionLine)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3DReadOnly pointOnLine = projectionLine.getPoint();
      Vector3DReadOnly directionOfLine = projectionLine.getDirection();
      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, directionOfLine);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (region.getConvexHull().isPointInside(intersectionInPlaneFrame.getX(), intersectionInPlaneFrame.getY()))
      {
         return intersectionWithPlane;
      }

      return null;
   }

   public static Point3D intersectRegionsWithRay(PlanarRegionsList regions, Point3D rayStart, Vector3D rayDirection)
   {
      double smallestDistance = Double.POSITIVE_INFINITY;
      Point3D closestIntersection = null;

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         Point3D intersection = intersectRegionWithRay(region, rayStart, rayDirection);
         if (intersection == null)
         {
            continue;
         }
         double distance = intersection.distance(rayStart);
         if (distance < smallestDistance)
         {
            smallestDistance = distance;
            closestIntersection = intersection;
         }
      }

      return closestIntersection;
   }

   public static Point3D intersectRegionWithRay(PlanarRegion region, Point3D rayStart, Vector3D rayDirection)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      region.getTransformToWorld(regionToWorld);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);
      planeNormal.applyTransform(regionToWorld);

      Point3D pointOnPlane = new Point3D();
      pointOnPlane.set(region.getConvexPolygon(0).getVertex(0));
      pointOnPlane.applyTransform(regionToWorld);

      Point3D intersectionWithPlane = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, rayStart, rayDirection);
      if (intersectionWithPlane == null)
      {
         return null;
      }

      Point3D intersectionInPlaneFrame = new Point3D(intersectionWithPlane);
      intersectionInPlaneFrame.applyInverseTransform(regionToWorld);
      // checking convex hull here - might be better to check all polygons to avoid false positive
      if (!region.getConvexHull().isPointInside(intersectionInPlaneFrame.getX(), intersectionInPlaneFrame.getY()))
      {
         return null;
      }

      Vector3D rayToIntersection = new Vector3D();
      rayToIntersection.sub(intersectionWithPlane, rayStart);
      if (rayToIntersection.dot(rayDirection) < 0.0)
      {
         return null;
      }

      return intersectionWithPlane;
   }

   public static boolean isPointOnRegion(PlanarRegion region, Point3D point, double epsilon)
   {
      Point3D closestPoint = closestPointOnPlane(point, region);
      return closestPoint.epsilonEquals(point, epsilon);
   }

   public static boolean isPointInsideAnyRegion(List<PlanarRegion> regions, Point3DReadOnly pointToCheck)
   {
      for (PlanarRegion region : regions)
      {
         if (isPointInsideARegion(region, pointToCheck))
         {
            return true;
         }
      }

      return false;
   }

   public static boolean isPointInsideARegion(PlanarRegion region, Point3DReadOnly pointToCheck)
   {
      ArrayList<Point2D> pointsInConcaveHull = new ArrayList<>();
      for (int i = 1; i < region.getConcaveHullSize(); i++)
      {
         Point2D point2D = (Point2D) region.getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         region.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         pointsInConcaveHull.add(new Point2D(fpt.getX(), fpt.getY()));
      }

      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(pointsInConcaveHull);

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointToCheck.getX(), centroid.getY() - pointToCheck.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(pointToCheck.getX() + directionToCentroid.getX(), pointToCheck.getY() + directionToCentroid.getY());

      boolean pointIsInside = VisibilityTools.isPointInsideConcavePolygon(pointsInConcaveHull.toArray(new Point2D[pointsInConcaveHull.size()]),
                                                                          new Point2D(pointToCheck.getX(), pointToCheck.getY()), endPoint);

      if (pointIsInside)
      {
         return true;
      }

      return false;
   }

   public static boolean isPartOfTheRegionInside(PlanarRegion regionToCheck, PlanarRegion containingRegion)
   {
      ArrayList<Point3D> pointsToCalculateCentroid = new ArrayList<>();
      Point2D[] homePointsArr = new Point2D[containingRegion.getConvexHull().getNumberOfVertices()];
      for (int i = 0; i < containingRegion.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) containingRegion.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         containingRegion.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);
         Point3D transformedPt = fpt.getPoint();

         homePointsArr[i] = new Point2D(transformedPt.getX(), transformedPt.getY());
         pointsToCalculateCentroid.add(new Point3D(transformedPt.getX(), transformedPt.getY(), transformedPt.getZ()));
      }

      ConvexPolygon2D homeConvexPol = new ConvexPolygon2D(homePointsArr);
      homeConvexPol.update();

      Point3D centroidOfHomeRegion = PointCloudTools.getCentroid(pointsToCalculateCentroid);

      Vector3D normal = calculateNormal(containingRegion);

      for (int i = 0; i < regionToCheck.getConvexHull().getNumberOfVertices(); i++)
      {
         Point2D point2D = (Point2D) regionToCheck.getConvexHull().getVertex(i);
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D fpt = new FramePoint3D();
         fpt.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         regionToCheck.getTransformToWorld(transToWorld);
         fpt.applyTransform(transToWorld);

         Point3D pointToProject = fpt.getPoint();
         Point3D projectedPointFromOtherRegion = new Point3D();

         //                     System.out.println(pointToProject + "  " + point3D + "  " + normal + "  " + projectedPointFromOtherRegion);
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point3D, normal, projectedPointFromOtherRegion);

         if (homeConvexPol.isPointInside(new Point2D(projectedPointFromOtherRegion.getX(), projectedPointFromOtherRegion.getY())))
         {
            return true;
         }
         //         }
      }

      return false;
   }

   public static Vector3D calculateNormal(PlanarRegion region)
   {
      Vector3D normal = new Vector3D();
      region.getNormal(normal);
      return normal;
   }

   public static ArrayList<PlanarRegion> determineWhichRegionsAreInside(PlanarRegion containingRegion, List<PlanarRegion> otherRegionsEx)
   {
      ArrayList<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();

      for (PlanarRegion otherRegion : otherRegionsEx)
      {
         if (PlanarRegionTools.isPartOfTheRegionInside(otherRegion, containingRegion))
         {
            regionsInsideHomeRegion.add(otherRegion);
         }
      }

      return regionsInsideHomeRegion;
   }

   public static boolean areAllPointsBelowTheRegion(PlanarRegion regionToCheck, PlanarRegion homeRegion)
   {
      for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
      {
         Point2D point2D = (Point2D) homeRegion.getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D homeRegionPoint = new FramePoint3D();
         homeRegionPoint.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transToWorld);
         homeRegionPoint.applyTransform(transToWorld);

         for (int j = 0; j < regionToCheck.getConcaveHull().length; j++)
         {
            Point2D point2D1 = (Point2D) regionToCheck.getConcaveHull()[j];
            Point3D point3D1 = new Point3D(point2D1.getX(), point2D1.getY(), 0);
            FramePoint3D otherRegionPoint = new FramePoint3D();
            otherRegionPoint.set(point3D1);
            RigidBodyTransform transToWorld1 = new RigidBodyTransform();
            regionToCheck.getTransformToWorld(transToWorld1);
            otherRegionPoint.applyTransform(transToWorld1);

            System.out.println(homeRegionPoint.getZ() + "   " + homeRegionPoint.getZ());
            if (homeRegionPoint.getZ() + 0.1 < otherRegionPoint.getZ())
            {
               return false;
            }
         }
      }

      return true;
   }

}
