package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Cluster
{
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final List<Point2D> rawPointsLocal = new ArrayList<>();
   private final List<Point2D> normalsInLocal = new ArrayList<>();
   private final List<Point2D> safeNormalsInLocal = new ArrayList<>();
   private final List<Point2D> navigableExtrusionsInLocal = new ArrayList<>();
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
         rawPointsLocal.add(rawPointsLocal.get(0));
         rawPointsLocal.add(rawPointsLocal.get(1));
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

   public List<Point3D> getUpdatedRawPoints()
   {
      return rawPointsLocal.stream().map(Point3D::new).collect(Collectors.toList());
   }

   public void setAdditionalExtrusionDistance(double extrusionDistance)
   {
      this.extrusionDistance = extrusionDistance;
   }

   public double getAdditionalExtrusionDistance()
   {
      return extrusionDistance;
   }

   public void addRawPointInLocal(Point2DReadOnly pointInLocal)
   {
      rawPointsLocal.add(new Point2D(pointInLocal));
      centroid.set(EuclidGeometryTools.averagePoint2Ds(rawPointsLocal));
   }

   public void addRawPointInWorld(Point3DReadOnly pointInWorld)
   {
      addRawPointInLocal(toLocal2D(pointInWorld));
   }

   public void addRawPointsInLocal(List<? extends Point2DReadOnly> pointsInLocal, boolean closed)
   {
      isObstacleClosed = closed;
      pointsInLocal.forEach(point -> rawPointsLocal.add(new Point2D(point)));

      if (closed)
      {
         rawPointsLocal.add(new Point2D(pointsInLocal.get(0)));

      }

      centroid.set(EuclidGeometryTools.averagePoint2Ds(rawPointsLocal));
   }

   public void addRawPointsInWorld(List<? extends Point3DReadOnly> pointsInWorld, boolean closed)
   {
      List<Point2D> pointsInLocal = pointsInWorld.stream().map(this::toLocal2D).collect(Collectors.toList());
      addRawPointsInLocal(pointsInLocal, closed);
   }

   public boolean isDynamic()
   {
      return isDynamic;
   }

   public void setDynamic(boolean dynamic)
   {
      isDynamic = dynamic;
   }

   public boolean isObstacleClosed()
   {
      return isObstacleClosed;
   }

   public int getNumberOfRawPoints()
   {
      return rawPointsLocal.size();
   }

   public Point2D getRawPointInLocal(int i)
   {
      return rawPointsLocal.get(i);
   }

   public Point2D getLastRawPointInLocal()
   {
      return rawPointsLocal.get(getNumberOfRawPoints() - 1);
   }

   public List<Point2D> getRawPointsInLocal()
   {
      return rawPointsLocal;
   }

   public Point3D getRawPointInWorld(int i)
   {
      return toWorld3D(rawPointsLocal.get(i));
   }

   public Point3D getLastRawPointInWorld()
   {
      return toWorld3D(getLastRawPointInLocal());
   }

   public List<Point3D> getRawPointsInWorld()
   {
      return rawPointsLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public void addNormalInLocal(Point2DReadOnly normalInLocal)
   {
      normalsInLocal.add(new Point2D(normalInLocal));
   }

   public void addNormalInWorld(Point3DReadOnly normalInWorld)
   {
      normalsInLocal.add(toLocal2D(normalInWorld));
   }

   public void addSafeNormalInLocal(Point2DReadOnly safeNormalInLocal)
   {
      safeNormalsInLocal.add(new Point2D(safeNormalInLocal));
   }

   public void addSafeNormalInWorld(Point3DReadOnly safeNormalInWorld)
   {
      safeNormalsInLocal.add(toLocal2D(safeNormalInWorld));
   }

   public void addNavigableExtrusionInLocal(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.add(new Point2D(navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionInWorld(Point3DReadOnly navigableExtrusionInWorld)
   {
      navigableExtrusionsInLocal.add(toLocal2D(navigableExtrusionInWorld));
   }

   public void addFirstNavigableExtrusionInLocal(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.add(0, new Point2D(navigableExtrusionInLocal));
   }

   public void addFirstNavigableExtrusionInWorld(Point3DReadOnly navigableExtrusionInWorld)
   {
      navigableExtrusionsInLocal.add(0, toLocal2D(navigableExtrusionInWorld));
   }

   public void addNonNavigableExtrusionPoint(Point2D point)
   {
      listOfNonNavigableExtrusions.add(point);
   }

   public void addFirstNonNavigableExtrusionPoint(Point2D point)
   {
      listOfNonNavigableExtrusions.add(0, point);
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusionsInLocal.size();
   }

   public Point2D getNavigableExtrusionInLocal(int i)
   {
      return navigableExtrusionsInLocal.get(i);
   }

   public List<Point2D> getNavigableExtrusionsInLocal()
   {
      return navigableExtrusionsInLocal;
   }
   
   public Point3D getNavigableExtrusionInWorld(int i)
   {
      return toWorld3D(getNavigableExtrusionInLocal(i));
   }
   
   public List<Point3D> getNavigableExtrusionsInWorld()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public List<Point2D> getListOfNonNavigableExtrusions()
   {
      return listOfNonNavigableExtrusions;
   }

   public int getNumberOfNormals()
   {
      return normalsInLocal.size();
   }

   public Point2D getNormalInLocal(int i)
   {
      return normalsInLocal.get(i);
   }

   public List<Point2D> getNormalsInLocal()
   {
      return normalsInLocal;
   }

   public Point3D getNormalInWorld(int i)
   {
      return toWorld3D(getNormalInLocal(i));
   }

   public List<Point3D> getNormalsInWorld()
   {
      return normalsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public int getNumberOfSafeNormals()
   {
      return safeNormalsInLocal.size();
   }

   public Point2D getSafeNormalInLocal(int i)
   {
      return safeNormalsInLocal.get(i);
   }

   public Point2D getLastSafeNormalInLocal()
   {
      return safeNormalsInLocal.get(getNumberOfSafeNormals() - 1);
   }

   public List<Point2D> getSafeNormalsInLocal()
   {
      return safeNormalsInLocal;
   }

   public Point3D getSafeNormalInWorld(int i)
   {
      return toWorld3D(getSafeNormalInLocal(i));
   }

   public Point3D getLastSafeNormalInWorld()
   {
      return toWorld3D(getSafeNormalInLocal(getNumberOfSafeNormals() - 1));
   }

   public List<Point3D> getSafeNormalsInWorld()
   {
      return safeNormalsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public boolean contains(Point2DReadOnly pointInLocal)
   {
      for (Point2D rawPoint : rawPointsLocal)
      {
         if (rawPoint.distance(pointInLocal) < 1E-3)
         {
            return true;
         }
      }

      return false;
   }

   private Point3D toWorld3D(Point2DReadOnly pointInLocal)
   {
      Point3D pointInWorld = new Point3D(pointInLocal);
      transformToWorld.transform(pointInWorld);
      return pointInWorld;
   }

   private Point3D toLocal3D(Point3DReadOnly pointInWorld)
   {
      Point3D pointInLocal = new Point3D();
      transformToWorld.inverseTransform(pointInWorld, pointInLocal);
      return pointInLocal;
   }

   private Point2D toLocal2D(Point3DReadOnly pointInWorld)
   {
      return new Point2D(toLocal3D(pointInWorld));
   }
}
