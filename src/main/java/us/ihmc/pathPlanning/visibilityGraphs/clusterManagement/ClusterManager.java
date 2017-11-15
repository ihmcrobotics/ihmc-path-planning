package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.paint.Color;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;

/*
 * TODO that class isn't very clear in general, we should talk about and review it.
 */
public class ClusterManager
{
   private final ArrayList<Cluster> listOfClusters = new ArrayList<>();

   public void addCluster(Cluster cluster)
   {
      listOfClusters.add(cluster);
   }

   public ArrayList<Cluster> getClusters()
   {
      return listOfClusters;
   }

   public void performExtrusions(Point2D initialObserver, double extrusionDistance)
   {
      for (Cluster cluster : listOfClusters)
      {
         ClusterTools.extrudeCluster(cluster, initialObserver, extrusionDistance, listOfClusters);
      }
   }

}
