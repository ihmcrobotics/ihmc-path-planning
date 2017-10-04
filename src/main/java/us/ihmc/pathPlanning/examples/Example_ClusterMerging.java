package us.ihmc.pathPlanning.examples;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.clusterManagement.Cluster;
import us.ihmc.pathPlanning.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.clusterManagement.ClusterMgr;
import us.ihmc.pathPlanning.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_ClusterMerging extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.20;
   
   ClusterMgr clusterMgr;

   public Example_ClusterMerging()
   {
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      clusterMgr = new ClusterMgr();
      clusterMgr.setVis(javaFXMultiColorMeshBuilder);
      
      createClosedSquare_OutsideExtrusion();
      createLine_Extrusion();

      clusterMgr.performExtrusions(new Point2D(), extrusionDistance);
      
      for(int i = 1; i < clusters.get(0).getListOfNavigableExtrusions().size(); i++)
      {
         Point2D from = clusters.get(0).getListOfNavigableExtrusions().get(i - 1);
         Point2D to = clusters.get(0).getListOfNavigableExtrusions().get(i);
         
         if(VisibilityTools.isPointVisible(from, to, clusters.get(1).getListOfNavigableExtrusions()))
         {
            javaFXMultiColorMeshBuilder.addLine(new Point3D(from.getX(), from.getY(), 0), new Point3D(to.getX(), to.getY(), 0), 0.035,
                                                Color.RED);
         }
      }
      
      
      for(Cluster cluster : clusters)
      {
         for (Point3D point : cluster.getRawPointsInCluster())
         {
            javaFXMultiColorMeshBuilder.addSphere(0.03f, point, Color.AQUAMARINE);
         }

         for (int i = 1; i < cluster.getRawPointsInCluster().size(); i++)
         {
            javaFXMultiColorMeshBuilder.addLine(cluster.getRawPointsInCluster().get(i - 1), cluster.getRawPointsInCluster().get(i), 0.005, Color.AQUAMARINE);
         }
         //      
//         for (Point3D point : cluster.getListOfSafeNormals())
//         {
//            javaFXMultiColorMeshBuilder.addSphere(0.03f, point, Color.WHITE);
//         }
         //
         //      for (Point2D point : cluster4.getListOfNonNavigableExtrusions())
         //      {
         //         javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(point.getX(), point.getY(), 0), Color.YELLOW);
         //      }
         //      
         for (int i = 1; i < cluster.getListOfNonNavigableExtrusions().size(); i++)
         {
            Point3D pt1 = new Point3D(cluster.getListOfNonNavigableExtrusions().get(i - 1).getX(),
                                      cluster.getListOfNonNavigableExtrusions().get(i - 1).getY(), 0);
            Point3D pt2 = new Point3D(cluster.getListOfNonNavigableExtrusions().get(i).getX(), cluster.getListOfNonNavigableExtrusions().get(i).getY(), 0);

            javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.005, Color.YELLOW);
         }
         
      }

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }
   
   private void createClosedSquare_InsideExtrusion()
   {
      Cluster cluster4 = new Cluster();
      clusterMgr.addCluster(cluster4);
      cluster4.setType(Type.POLYGON);
      
      cluster4.addRawPoint(new Point3D(-0.975,  0.475 + 5,  0.000));
      cluster4.addRawPoint(new Point3D(0.975,  0.475 + 5,  0.000));
      cluster4.addRawPoint(new Point3D(0.975, -0.475 + 5,  0.000));
      cluster4.addRawPoint(new Point3D(-0.975, -0.475 + 5,  0.000));
      
      cluster4.setClusterClosure(true);
      cluster4.setExtrusionSide(ExtrusionSide.INSIDE);
      clusters.add(cluster4);
   }
   
   private void createClosedSquare_OutsideExtrusion()
   {
      Cluster cluster4 = new Cluster();
      clusterMgr.addCluster(cluster4);
      cluster4.setType(Type.POLYGON);
      
      cluster4.addRawPoint(new Point3D(-0.975,  0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(0.975,  0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(0.975, -0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(-0.975, -0.475 + 2,  0.000));
      
      cluster4.setClusterClosure(true);
      cluster4.setExtrusionSide(ExtrusionSide.OUTSIDE);
      clusters.add(cluster4);
   }
   
   private void createOpenSquare_Extrusion()
   {
      Cluster cluster4 = new Cluster();
      clusterMgr.addCluster(cluster4);
      cluster4.setType(Type.POLYGON);
      
      cluster4.addRawPoint(new Point3D(-0.975 + 3,  0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(0.975 + 3,  0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(0.975 + 3, -0.475 + 2,  0.000));
      cluster4.addRawPoint(new Point3D(-0.975 + 3, -0.475 + 2,  0.000));
      
      clusters.add(cluster4);
   }
   
   private void createLine_Extrusion()
   {
      Cluster cluster4 = new Cluster();
      clusterMgr.addCluster(cluster4);
      cluster4.setType(Type.LINE);
      
      cluster4.addRawPoint(new Point3D(-0.975 + 1,  0.475 + 1.5,  0.000));
      cluster4.addRawPoint(new Point3D(0.975 + 1,  0.475 + 1.5,  0.000));
      
      clusters.add(cluster4);
   }
   
   public static void main(String[] args)
   {
      launch();
   }

}