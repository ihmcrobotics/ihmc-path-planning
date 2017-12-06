package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class ClusterMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private final Group rawPointsGroup = new Group();
   private final Group navigableExtrusionsGroup = new Group();
   private final Group nonNavigableExtrusionsGroup = new Group();
   private final AtomicReference<Map<Integer, MeshView>> rawPointsToRenderReference = new AtomicReference<>(null);
   private Map<Integer, MeshView> rawPointsRendered;
   private final AtomicReference<Map<Integer, MeshView>> navigableExtrusionsToRenderReference = new AtomicReference<>(null);
   private Map<Integer, MeshView> navigableExtrusionsRendered;
   private final AtomicReference<Map<Integer, MeshView>> nonNavigableExtrusionsToRenderReference = new AtomicReference<>(null);
   private Map<Integer, MeshView> nonNavigableExtrusionsRendered;

   private final AtomicReference<Boolean> resetRequested;
   private final AtomicReference<Boolean> showRawPoints;
   private final AtomicReference<Boolean> showNavigableExtrusions;
   private final AtomicReference<Boolean> closeNavigableExtrusions;
   private final AtomicReference<Boolean> showNonNavigableExtrusions;
   private final AtomicReference<Boolean> closeNonNavigableExtrusions;

   public ClusterMeshViewer(REAMessager messager)
   {
      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      showRawPoints = messager.createInput(UIVisibilityGraphsTopics.ShowClusterRawPoints, false);
      showNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, false);
      closeNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.CloseClusterNavigableExtrusions, false);
      showNonNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, false);
      closeNonNavigableExtrusions = messager.createInput(UIVisibilityGraphsTopics.CloseClusterNonNavigableExtrusions, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, this::handleShowClusterNavigableExtrusionsThreadSafe);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, this::handleShowClusterNonNavigableExtrusionsThreadSafe);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowClusterRawPoints, this::handleShowClusterRawPointsThreadSafe);
      root.setMouseTransparent(true);
      root.getChildren().addAll(rawPointsGroup, navigableExtrusionsGroup);
   }

   private void handleShowClusterRawPointsThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShowClusterRawPoints(show);
      else
         Platform.runLater(() -> handleShowClusterRawPoints(show));
   }

   private void handleShowClusterRawPoints(boolean show)
   {
      if (!show)
         rawPointsGroup.getChildren().clear();
      else if (rawPointsRendered != null)
         rawPointsGroup.getChildren().addAll(rawPointsRendered.values());
   }

   private void handleShowClusterNavigableExtrusionsThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShowNavigableExtrusionsPoints(show);
      else
         Platform.runLater(() -> handleShowNavigableExtrusionsPoints(show));
   }

   private void handleShowNavigableExtrusionsPoints(boolean show)
   {
      if (!show)
         navigableExtrusionsGroup.getChildren().clear();
      else if (navigableExtrusionsRendered != null)
         navigableExtrusionsGroup.getChildren().addAll(navigableExtrusionsRendered.values());
   }

   private void handleShowClusterNonNavigableExtrusionsThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShowNonNavigableExtrusionsPoints(show);
      else
         Platform.runLater(() -> handleShowNonNavigableExtrusionsPoints(show));
   }

   private void handleShowNonNavigableExtrusionsPoints(boolean show)
   {
      if (!show)
         nonNavigableExtrusionsGroup.getChildren().clear();
      else if (nonNavigableExtrusionsRendered != null)
         nonNavigableExtrusionsGroup.getChildren().addAll(nonNavigableExtrusionsRendered.values());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         rawPointsGroup.getChildren().clear();
         rawPointsToRenderReference.getAndSet(null);
         navigableExtrusionsGroup.getChildren().clear();
         navigableExtrusionsToRenderReference.getAndSet(null);
         nonNavigableExtrusionsGroup.getChildren().clear();
         nonNavigableExtrusionsToRenderReference.getAndSet(null);
         return;
      }

      Map<Integer, MeshView> rawPointsToRender = rawPointsToRenderReference.getAndSet(null);

      if (rawPointsToRender != null)
      {
         rawPointsRendered = rawPointsToRender;
         if (showRawPoints.get())
         {
            rawPointsGroup.getChildren().clear();
            rawPointsGroup.getChildren().addAll(rawPointsToRender.values());
         }
      }

      Map<Integer, MeshView> navigableExtrusionsRender = navigableExtrusionsToRenderReference.getAndSet(null);

      if (navigableExtrusionsRender != null)
      {
         navigableExtrusionsRendered = navigableExtrusionsRender;
         if (showNavigableExtrusions.get())
         {
            navigableExtrusionsGroup.getChildren().clear();
            navigableExtrusionsGroup.getChildren().addAll(navigableExtrusionsRender.values());
         }
      }

      Map<Integer, MeshView> nonNavigableExtrusionsRender = nonNavigableExtrusionsToRenderReference.getAndSet(null);

      if (nonNavigableExtrusionsRender != null)
      {
         nonNavigableExtrusionsRendered = nonNavigableExtrusionsRender;
         if (showNonNavigableExtrusions.get())
         {
            nonNavigableExtrusionsGroup.getChildren().clear();
            nonNavigableExtrusionsGroup.getChildren().addAll(nonNavigableExtrusionsRender.values());
         }
      }
   }

   public void processNavigableRegions(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> rawPointsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> navigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> nonNavigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, Material> materials = new HashMap<>();

      for (NavigableRegion navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getRegionId();
         JavaFXMeshBuilder rawPointsMeshBuilder = getOrCreate(rawPointsMeshBuilders, regionId);
         JavaFXMeshBuilder navigableExtrusionsMeshBuilder = getOrCreate(navigableExtrusionsMeshBuilders, regionId);
         JavaFXMeshBuilder nonNavigableExtrusionsMeshBuilder = getOrCreate(nonNavigableExtrusionsMeshBuilders, regionId);

         List<Cluster> clusters = navigableRegionLocalPlanner.getClusters();
         for (Cluster cluster : clusters)
         {
            for (Point3D rawPoint : cluster.getRawPointsInWorld())
               rawPointsMeshBuilder.addTetrahedron(VisualizationParameters.CLUSTER_RAWPOINT_SIZE, rawPoint);
            navigableExtrusionsMeshBuilder.addMultiLine(cluster.getNavigableExtrusionsInWorld(), VisualizationParameters.NAVIGABLECLUSTER_LINE_THICKNESS,
                                                        closeNavigableExtrusions.get());
            nonNavigableExtrusionsMeshBuilder.addMultiLine(cluster.getNonNavigableExtrusionsInWorld(), VisualizationParameters.NON_NAVIGABLECLUSTER_LINE_THICKNESS,
                                                           closeNonNavigableExtrusions.get());
         }

         materials.put(regionId, new PhongMaterial(getLineColor(regionId)));
      }

      HashMap<Integer, MeshView> rawPointsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> navigableExtrusionsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> nonNavigableExtrusionsMapToRender = new HashMap<>();

      for (Integer id : rawPointsMeshBuilders.keySet())
      {
         MeshView rawPointsMeshView = new MeshView(rawPointsMeshBuilders.get(id).generateMesh());
         rawPointsMeshView.setMaterial(materials.get(id));
         rawPointsMapToRender.put(id, rawPointsMeshView);

         MeshView navigableExtrusionsMeshView = new MeshView(navigableExtrusionsMeshBuilders.get(id).generateMesh());
         navigableExtrusionsMeshView.setMaterial(materials.get(id));
         navigableExtrusionsMapToRender.put(id, navigableExtrusionsMeshView);

         MeshView nonNavigableExtrusionsMeshView = new MeshView(nonNavigableExtrusionsMeshBuilders.get(id).generateMesh());
         nonNavigableExtrusionsMeshView.setMaterial(materials.get(id));
         nonNavigableExtrusionsMapToRender.put(id, nonNavigableExtrusionsMeshView);
      }

      rawPointsToRenderReference.set(rawPointsMapToRender);
      navigableExtrusionsToRenderReference.set(navigableExtrusionsMapToRender);
      nonNavigableExtrusionsToRenderReference.set(nonNavigableExtrusionsMapToRender);
   }

   private JavaFXMeshBuilder getOrCreate(Map<Integer, JavaFXMeshBuilder> meshBuilders, int regionId)
   {
      JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
      if (meshBuilder == null)
      {
         meshBuilder = new JavaFXMeshBuilder();
         meshBuilders.put(regionId, meshBuilder);
      }
      return meshBuilder;
   }

   private Color getLineColor(int regionId)
   {
      return VizGraphsPlanarRegionViewer.getRegionColor(regionId).darker();
   }

   public Node getRoot()
   {
      return root;
   }
}
