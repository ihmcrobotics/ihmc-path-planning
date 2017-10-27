package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class Cluster
{
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private List<Point3D> listOfRawPoints = new ArrayList<>();
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

   public void setTransformToWorld(RigidBodyTransform transform)
   {
      transformToWorld.set(transform);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
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
      return listOfNavigableExtrusions.stream().map(Point2D::new).collect(Collectors.toList());
   }

   public List<Point3D> getUpdatedRawPoints()
   {
      return listOfRawPoints.stream().map(Point3D::new).collect(Collectors.toList());
   }

   public List<Point3D> getUpdatedNormals()
   {
      return listOfNormals.stream().map(Point3D::new).collect(Collectors.toList());
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
      listOfNavigableExtrusions.add(new Point2D(point));
   }

   public void addFirstNavigableExtrusionPoint(Point3D point)
   {
      listOfNavigableExtrusions.add(0, new Point2D(point));
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
      return listOfNormalsSafe.stream().map(Point3D::new).collect(Collectors.toList());
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
