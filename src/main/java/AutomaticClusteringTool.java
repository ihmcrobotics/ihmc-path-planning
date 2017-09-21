


import java.util.ArrayList;

import newz.Cluster;
import us.ihmc.euclid.tuple3D.Point3D;

public class AutomaticClusteringTool
{
   public static ArrayList<Cluster> createClusters(ArrayList<Point3D> listOfPoints, double clusteringDistanceThreshold)
   {
      ArrayList<Cluster> listOfClusters = new ArrayList<>();

      Cluster cluster = new Cluster();

      for (int i = 0; i < listOfPoints.size() - 1; i++)
      {
         cluster.addRawPoint(listOfPoints.get(i));

         if (listOfPoints.get(i).distance(listOfPoints.get(i + 1)) > clusteringDistanceThreshold)
         {
            if (cluster.getRawPointsInCluster().size() > 1)
            {
               listOfClusters.add(cluster);
            }
            cluster = new Cluster();
         }

         if (i == listOfPoints.size() - 2)
         {
            cluster.addRawPoint(listOfPoints.get(listOfPoints.size() - 1));

            if (cluster.getRawPointsInCluster().size() > 1)
            {
               listOfClusters.add(cluster);
            }

         }
      }
      
      
//      boolean firstTime = true;
//      Cluster cluster = null;
//      int j = 0;
//      
//      for (int i = 0; i < listOfPoints.size(); i++)
//      {
////       System.out.println(i);
//
//         if (i < listOfPoints.size() - 1)
//         {
////          System.out.println("Comparing distance between " + i + " and " + (i + 1));
//            double distance = listOfPoints.get(i + 1).distance(listOfPoints.get(i));
//
//            if (distance < clusteringDistanceThreshold)
//            {
////             System.out.println("line " + initialPoint + "   " + finalPoint);
////             System.out.println(firstTime);
//
//               if (firstTime)
//               {
////                System.out.println("\n\n\nNew cluster");
//                  firstTime = false;
//                  cluster = new Cluster();
//                  j++;
//                  listOfClusters.add(cluster);
//                  cluster.addRawPoint(listOfPoints.get(i));
//
////                System.out.println("Adding point" + i + " to cluster ");
//               }
//               else
//               {
//                  if (!cluster.contains(listOfPoints.get(i)))
//                  {
////                   System.out.println("Adding point" + i + " to cluster ");
//                     cluster.addRawPoint(listOfPoints.get(i));
//                  }
//
//                  if (!cluster.contains(listOfPoints.get(i + 1)))
//                  {
////                   System.out.println("Adding point" + (i + 1) + " to cluster ");
//                     cluster.addRawPoint(listOfPoints.get(i + 1));
//                  }
//               }
//            }
//            else
//            {
//               firstTime = true;
//            }
//         }
//      }

      System.out.println("Calculated a total of " + listOfClusters.size() + " clusters");
//
//      for (int i = 0; i < listOfClusters.size(); i++)
//      {
//         System.out.println("     Cluster - " + i + " has size of " + listOfClusters.get(i).getPointsInCluster().size());
//      }

      return listOfClusters;
   }
     
   public static ArrayList<Cluster> createClusters2(ArrayList<Point3D> listOfPoints, double clusteringDistanceThreshold)
   {
      ArrayList<Cluster> listOfClusters = new ArrayList<>();
      
      Cluster cluster = new Cluster();
      
      for(int i = 0; i < listOfPoints.size() - 1; i++)
      {
         cluster.addRawPoint(listOfPoints.get(i));
         
         if(listOfPoints.get(i).distance(listOfPoints.get(i+1)) > 1.0)
         {
            if(cluster.getRawPointsInCluster().size() > 1)
            {
               listOfClusters.add(cluster);
            }
            cluster = new Cluster();
         }
         
         if(i == listOfPoints.size() - 2)
         {
            cluster.addRawPoint(listOfPoints.get(listOfPoints.size() - 1));
            
            if(cluster.getRawPointsInCluster().size() > 1)
            {
               listOfClusters.add(cluster);
            }

         }
      }
      
      
//      boolean firstTime = true;
//      Cluster cluster = null;
//      int j = 0;
//      
//      for (int i = 0; i < listOfPoints.size(); i++)
//      {
////       System.out.println(i);
//
//         if (i < listOfPoints.size() - 1)
//         {
////          System.out.println("Comparing distance between " + i + " and " + (i + 1));
//            double distance = listOfPoints.get(i + 1).distance(listOfPoints.get(i));
//
//            if (distance < clusteringDistanceThreshold)
//            {
////             System.out.println("line " + initialPoint + "   " + finalPoint);
////             System.out.println(firstTime);
//
//               if (firstTime)
//               {
////                System.out.println("\n\n\nNew cluster");
//                  firstTime = false;
//                  cluster = new Cluster();
//                  j++;
//                  listOfClusters.add(cluster);
//                  cluster.addRawPoint(listOfPoints.get(i));
//
////                System.out.println("Adding point" + i + " to cluster ");
//               }
//               else
//               {
//                  if (!cluster.contains(listOfPoints.get(i)))
//                  {
////                   System.out.println("Adding point" + i + " to cluster ");
//                     cluster.addRawPoint(listOfPoints.get(i));
//                  }
//
//                  if (!cluster.contains(listOfPoints.get(i + 1)))
//                  {
////                   System.out.println("Adding point" + (i + 1) + " to cluster ");
//                     cluster.addRawPoint(listOfPoints.get(i + 1));
//                  }
//               }
//            }
//            else
//            {
//               firstTime = true;
//            }
//         }
//      }

      System.out.println("Calculated a total of " + listOfClusters.size() + " clusters");
//
//      for (int i = 0; i < listOfClusters.size(); i++)
//      {
//         System.out.println("     Cluster - " + i + " has size of " + listOfClusters.get(i).getPointsInCluster().size());
//      }

      return listOfClusters;
   }

}
