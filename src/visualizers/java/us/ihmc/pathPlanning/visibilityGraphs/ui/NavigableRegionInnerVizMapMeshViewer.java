package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionLocalPlanner;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class NavigableRegionInnerVizMapMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private AtomicReference<Map<Integer, MeshView>> regionVisMapToRenderReference = new AtomicReference<>(null);
   private Map<Integer, MeshView> regionVisMapRendered;

   private final REAMessager messager;
   private final AtomicReference<Boolean> resetRequested;
   private final AtomicReference<Boolean> show;

   private final double lineWidth = 0.0025;

   public NavigableRegionInnerVizMapMeshViewer(REAMessager messager)
   {
      this.messager = messager;
      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      show = messager.createInput(UIVisibilityGraphsTopics.ShowLocalGraphs, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowLocalGraphs, this::handleShowThreadSafe);
      root.setMouseTransparent(true);
   }

   private void handleShowThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShow(show);
      else
         Platform.runLater(() -> handleShow(show));
   }

   private void handleShow(boolean show)
   {
      if (!show)
         root.getChildren().clear();
      else
         root.getChildren().addAll(regionVisMapRendered.values());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         root.getChildren().clear();
         regionVisMapToRenderReference.getAndSet(null);
         return;
      }

      Map<Integer, MeshView> regionVisMapToRender = regionVisMapToRenderReference.getAndSet(null);

      if (regionVisMapToRender != null)
      {
         regionVisMapRendered = regionVisMapToRender;

         if (show.get())
         {
            root.getChildren().clear();
            root.getChildren().addAll(regionVisMapToRender.values());
         }
      }
   }

   public void processNavigableRegions(List<NavigableRegionLocalPlanner> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> meshBuilders = new HashMap<>();
      Map<Integer, Material> materials = new HashMap<>();

      for (NavigableRegionLocalPlanner navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getRegionId();
         JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
         if (meshBuilder == null)
         {
            meshBuilder = new JavaFXMeshBuilder();
            meshBuilders.put(regionId, meshBuilder);
         }

         RigidBodyTransform transformToWorld = navigableRegionLocalPlanner.getLocalReferenceFrame().getTransformToWorldFrame();
         SimpleWeightedGraph<Point2D, DefaultWeightedEdge> localVisibilityGraph = navigableRegionLocalPlanner.getLocalVisibilityGraph();

         for (DefaultWeightedEdge edge : localVisibilityGraph.edgeSet())
         {
            Point3D edgeSource = toWorld(localVisibilityGraph.getEdgeSource(edge), transformToWorld);
            Point3D edgeTarget = toWorld(localVisibilityGraph.getEdgeTarget(edge), transformToWorld);
            meshBuilder.addLine(edgeSource, edgeTarget, lineWidth);
         }

         materials.put(regionId, new PhongMaterial(getLineColor(regionId)));
      }

      HashMap<Integer, MeshView> regionVisMapToRender = new HashMap<>();

      for (Integer id : meshBuilders.keySet())
      {
         MeshView meshView = new MeshView(meshBuilders.get(id).generateMesh());
         meshView.setMaterial(materials.get(id));
         regionVisMapToRender.put(id, meshView);
      }

      regionVisMapToRenderReference.set(regionVisMapToRender);
   }

   private Color getLineColor(int regionId)
   {
      return VizGraphsPlanarRegionViewer.getRegionColor(regionId).brighter();
   }

   private Point3D toWorld(Point2D localPoint, RigidBodyTransform transformToWorld)
   {
      Point3D worldPoint = new Point3D(localPoint);
      transformToWorld.transform(worldPoint);
      return worldPoint;
   }

   public Node getRoot()
   {
      return root;
   }
}
