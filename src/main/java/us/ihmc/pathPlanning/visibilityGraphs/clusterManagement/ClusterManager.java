package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;

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
