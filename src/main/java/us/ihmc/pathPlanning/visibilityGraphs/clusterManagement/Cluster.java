package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class Cluster
{
   private Point3D originPosition = new Point3D(); // FIXME this field seems to never be set but induce some additional computation.
   private List<Point3D> listOfRawPoints = new ArrayList<>();
   private final List<Point3D> listOfVertices = new ArrayList<>();
   private final List<Point3D> listOfNormals = new ArrayList<>();
   private final List<Point3D> listOfNormalsSafe = new ArrayList<>();
   private final List<Point3D> listOfCorrectNormals = new ArrayList<>();
   private final List<Point2D> listOfNavigableExtrusions = new ArrayList<>();
   private final List<Point2D> listOfNonNavigableExtrusions = new ArrayList<>();

   // TODO Provide some info about the usage/meaning of these fields
   private boolean isObstacleClosed = false;
   private double extrusionDistance = 0.0;
   private boolean isDynamic = false;
   private String name;
   private Point2D observer;
   private RigidBodyTransform transform;
   private Point3D centroid = new Point3D();

   public enum ExtrusionSide
   {
      AUTO, INSIDE, OUTSIDE
   };

   private ExtrusionSide extrusionSide = ExtrusionSide.AUTO;

   public enum Type
   {
      LINE, POLYGON
   };

   private Type type = Type.POLYGON;

   public Cluster()
   {
   }

   public Cluster(List<Point3D> listOfRawPoints, boolean closed)
   {
      this.listOfRawPoints = listOfRawPoints;
      isObstacleClosed = closed;

      if (closed)
      {
         listOfRawPoints.add(listOfRawPoints.get(0));
         listOfRawPoints.add(listOfRawPoints.get(1));
      }

      centroid = calculateCentroid();
   }

   public void setExtrusionSide(ExtrusionSide extrusionSide)
   {
      this.extrusionSide = extrusionSide;
   }

   public ExtrusionSide getExtrusionSide()
   {
      return extrusionSide;
   }

   public void setClusterClosure(boolean closed)
   {
      if (closed && !this.isObstacleClosed)
      {
         listOfRawPoints.add(listOfRawPoints.get(0));
         listOfRawPoints.add(listOfRawPoints.get(1));
      }

      this.isObstacleClosed = closed;
   }

   public void setType(Type type)
   {
      this.type = type;
   }

   public Type getType()
   {
      return type;
   }

   public Cluster(List<Point3D> listOfRawPoints, boolean closed, boolean isDynamic, Point2D observer, String name)
   {
      isObstacleClosed = closed;
      this.isDynamic = isDynamic;
      this.observer = observer;
      this.listOfRawPoints.addAll(listOfRawPoints);
      this.name = name;

      if (closed)
      {
         this.listOfRawPoints.add(this.listOfRawPoints.get(0));
         this.listOfRawPoints.add(this.listOfRawPoints.get(1));

      }
      centroid = calculateCentroid();
   }

   private Point3D calculateCentroid()
   {
      return EuclidGeometryTools.averagePoint3Ds(listOfRawPoints);
   }

   public Point3D getCentroid()
   {
      return centroid;
   }

   public void setTransform(RigidBodyTransform t)
   {
      transform = t;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public void setObserver(Point2D observer)
   {
      this.observer = observer;
   }

   public Point2D getObserver()
   {
      return observer;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public List<Point2D> getUpdatedNavigableExtrusions()
   {
      List<Point2D> list = new ArrayList<>();
      for (Point2D point : listOfNavigableExtrusions)
      {
         list.add(new Point2D(point.getX() + originPosition.getX(), point.getY() + originPosition.getY()));
      }

      return list;
   }

   public List<Point3D> getUpdatedRawPoints()
   {
      List<Point3D> list = new ArrayList<>();
      for (Point3D point : listOfRawPoints)
      {
         list.add(new Point3D(point.getX() + originPosition.getX(), point.getY() + originPosition.getY(), point.getZ() + originPosition.getZ()));
      }

      return list;
   }

   public List<Point3D> getUpdatedNormals()
   {
      List<Point3D> list = new ArrayList<>();
      for (Point3D point : listOfNormals)
      {
         list.add(new Point3D(point.getX() + originPosition.getX(), point.getY() + originPosition.getY(), point.getZ() + originPosition.getZ()));
      }

      return list;
   }

   public void updatePosition(Point3D point)
   {
      originPosition = point;
   }

   public Point3D getOriginPosition()
   {
      return originPosition;
   }

   public void setAdditionalExtrusionDistance(double extrusionDistance)
   {
      this.extrusionDistance = extrusionDistance;
   }

   public double getAdditionalExtrusionDistance()
   {
      return extrusionDistance;
   }

   public void addRawPoint(Point3D point)
   {
      listOfRawPoints.add(point);
      centroid = calculateCentroid();
   }

   public boolean isDynamic()
   {
      return isDynamic;
   }

   public void setDynamic(boolean dynamic)
   {
      isDynamic = dynamic;
   }

   public void addRawPoints(List<Point3D> points, boolean closed)
   {
      isObstacleClosed = closed;
      listOfRawPoints.addAll(points);

      if (closed)
      {
         listOfRawPoints.add(points.get(0));

      }

      centroid = calculateCentroid();
   }

   public boolean isObstacleClosed()
   {
      return isObstacleClosed;
   }

   public List<Point3D> getRawPointsInCluster()
   {
      return listOfRawPoints;
   }

   public void addVertex(Point3D vertex)
   {
      listOfVertices.add(vertex);
   }

   public List<Point3D> getListOfVertices()
   {
      return listOfVertices;
   }

   public void addNormal(Point3D normal)
   {
      listOfNormals.add(normal);
   }

   public void addSafeNormal(Point3D normal)
   {
      listOfNormalsSafe.add(normal);
   }

   public void addCorrectNormalPoint(Point3D normal)
   {
      listOfCorrectNormals.add(normal);
   }

   public void addNavigableExtrusionPoint(Point3D point)
   {
      listOfNavigableExtrusions.add(new Point2D(point.getX(), point.getY()));
   }

   public void addFirstNavigableExtrusionPoint(Point3D point)
   {
      listOfNavigableExtrusions.add(0, new Point2D(point.getX(), point.getY()));
   }

   public void addNonNavigableExtrusionPoint(Point2D point)
   {
      listOfNonNavigableExtrusions.add(point);
   }

   public void addFirstNonNavigableExtrusionPoint(Point2D point)
   {
      listOfNonNavigableExtrusions.add(0, point);
   }

   public List<Point2D> getListOfNavigableExtrusions()
   {
      return listOfNavigableExtrusions;
   }

   public List<Point2D> getListOfNonNavigableExtrusions()
   {
      return listOfNonNavigableExtrusions;
   }

   public List<Point3D> getListOfSafeNormals()
   {
      return listOfNormalsSafe;
   }

   public List<Point3D> getUpdatedListOfSafeNormals()
   {
      List<Point3D> list = new ArrayList<>();
      for (Point3D point : listOfNormalsSafe)
      {
         list.add(new Point3D(point.getX() + originPosition.getX(), point.getY() + originPosition.getY(), point.getZ() + originPosition.getZ()));
      }

      return list;
   }

   public List<Point3D> getListOfCorrectNormals()
   {
      return listOfCorrectNormals;
   }

   public List<Point3D> getNormals()
   {
      return new ArrayList<>(listOfNormals);
   }

   public boolean contains(Point3D point)
   {
      for (Point3D point1 : listOfRawPoints)
      {
         if (point1.distance(point) < 1E-3)
         {
            return true;
         }
      }

      return false;
   }
}
