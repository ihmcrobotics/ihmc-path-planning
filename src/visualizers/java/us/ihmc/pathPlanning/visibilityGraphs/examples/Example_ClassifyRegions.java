package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_ClassifyRegions extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   ArrayList<PlanarRegion> accesibleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> obstacleRegions = new ArrayList<>();

   public Example_ClassifyRegions()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      loadPointCloudFromFile("PlanarRegions_201709260229.txt");
      classifyRegions(regions);
      
      for (PlanarRegion region : accesibleRegions)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         region.getTransformToWorld(transform);

         Color color = Color.GREEN;
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
         {
            javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
         }
      }

      for (PlanarRegion region : obstacleRegions)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         region.getTransformToWorld(transform);

         Color color = Color.RED;
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
         {
            javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
         }
      }

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private void classifyRegions(ArrayList<PlanarRegion> regions)
   {
      for (PlanarRegion region : regions)
      {
         Vector3D normal = calculateNormal(region);

         if (normal != null)
         {
            if (Math.abs(normal.getZ()) < 0.5)
            {
               obstacleRegions.add(region);
               Cluster cluster = new Cluster();
               clusters.add(cluster);
               cluster.setObserverInLocal(clusters.get(0).getCentroidInLocal());

               for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
               {
                  Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
                  Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
                  FramePoint3D fpt = new FramePoint3D();
                  fpt.set(point3D);
                  RigidBodyTransform transToWorld = new RigidBodyTransform();
                  region.getTransformToWorld(transToWorld);
                  fpt.applyTransform(transToWorld);
                  Point3D pointToProject = fpt.getPoint();

                  cluster.addRawPointInWorld(pointToProject);
               }
            }
            else
            {
               accesibleRegions.add(region);
            }
         }
      }
   }

   private Vector3D calculateNormal(PlanarRegion region)
   {
      if (!region.getConvexHull().isEmpty())
      {
         FramePoint3D fpt1 = new FramePoint3D();
         fpt1.set(new Point3D(region.getConvexHull().getVertex(0).getX(), region.getConvexHull().getVertex(0).getY(), 0));
         RigidBodyTransform trans = new RigidBodyTransform();
         region.getTransformToWorld(trans);
         fpt1.applyTransform(trans);

         FramePoint3D fpt2 = new FramePoint3D();
         fpt2.set(new Point3D(region.getConvexHull().getVertex(1).getX(), region.getConvexHull().getVertex(1).getY(), 0));
         fpt2.applyTransform(trans);

         FramePoint3D fpt3 = new FramePoint3D();
         fpt3.set(new Point3D(region.getConvexHull().getVertex(2).getX(), region.getConvexHull().getVertex(2).getY(), 0));
         fpt3.applyTransform(trans);

         Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(fpt1.getPoint(), fpt2.getPoint(), fpt3.getPoint());
         return normal;
      }
      return null;
   }

   public ArrayList<Point3D> loadPointCloudFromFile(String fileName)
   {
      ArrayList<Cluster> clusters = new ArrayList<>();
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         //br = new BufferedReader(new FileReader(FILENAME));
         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         double averageX = 0.0;
         double averageY = 0.0;
         double averageZ = 0.0;

         int index = 0;

         Cluster cluster = new Cluster();
         int nPacketsRead = 0;

         ArrayList<Point3D> pointsTemp = new ArrayList<>();

         while ((sCurrentLine = br.readLine()) != null)
         {
            //            System.out.println(sCurrentLine);

            if (sCurrentLine.contains("PR_"))
            {
               if (!pointsTemp.isEmpty())
               {
                  cluster.addRawPointsInWorld(pointsTemp, true);
                  pointsTemp.clear();
               }

               cluster = new Cluster();
               clusters.add(cluster);
               nPacketsRead++;
               //               System.out.println("New cluster created");
            }

            else if (sCurrentLine.contains("RBT,"))
            {
               //               System.out.println("Transformation read");
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());

               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);
               Vector3D translation = new Vector3D(x, y, z);

               float qx = (float) Double.parseDouble(points[3]);
               float qy = (float) Double.parseDouble(points[4]);
               float qz = (float) Double.parseDouble(points[5]);
               float qs = (float) Double.parseDouble(points[6]);
               Quaternion quat = new Quaternion(qx, qy, qz, qs);

               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(quat, translation);
               cluster.setTransformToWorld(rigidBodyTransform);
            }
            else
            {
               //               System.out.println("adding point");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);

               pointsTemp.add(new Point3D(x, y, z));

               averageX = averageX + x;
               averageY = averageY + y;
               averageZ = averageZ + z;

               index++;
            }
         }

         for (Cluster cluster1 : clusters)
         {
            ArrayList<Point2D> vertices = new ArrayList<>();

            for (Point3D pt : cluster1.getRawPointsInWorld())
            {
               vertices.add(new Point2D(pt.getX(), pt.getY()));
            }

            ConvexPolygon2D convexPolygon = new ConvexPolygon2D(vertices);

            PlanarRegion planarRegion = new PlanarRegion(cluster1.getTransformToWorld(), convexPolygon);

            regions.add(planarRegion);
         }

         System.out.println("Loaded " + regions.size() + " regions");
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();
         }

      }
      return null;

   }

   public static void main(String[] args)
   {
      launch();
   }
}