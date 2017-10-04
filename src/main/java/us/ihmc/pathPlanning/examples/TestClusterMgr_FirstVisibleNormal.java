package us.ihmc.pathPlanning.examples;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.clusterManagement.Cluster;
import us.ihmc.pathPlanning.clusterManagement.ClusterMgr;

public class TestClusterMgr_FirstVisibleNormal extends Application
{
   
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      
      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette );
      
      Cluster cluster = new Cluster();
      cluster.addRawPoint(new Point3D(1, 1, 0));
      cluster.addRawPoint(new Point3D(2, 1, 0));
      cluster.addRawPoint(new Point3D(3, 1, 0));
      cluster.addRawPoint(new Point3D(4, 1, 0));

      ClusterMgr clusterMgr = new ClusterMgr();
      clusterMgr.addCluster(cluster);

      clusterMgr.generateNormalsFromRawBoundaryMap(0.25);
      int i = clusterMgr.determineExtrusionSide(cluster, new Point2D(0, 0));

      javaFXMultiColorMeshBuilder.addSphere(0.1f, new Point3D(), Color.GREEN);

      for (Point3D point : cluster.getRawPointsInCluster())
      {
         javaFXMultiColorMeshBuilder.addSphere(0.1f, point, Color.BISQUE);
      }

      if (i % 2 == 0)
      {
         i = 0;
      }
      else
      {
         i = 1;
      }

      for (int j = i; j < cluster.getListOfSafeNormals().size(); j = j + 2)
      {
         javaFXMultiColorMeshBuilder.addSphere(0.1f, cluster.getListOfSafeNormals().get(j), Color.RED);
      }
      
      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String args[])
   {
      launch();
   }

}
