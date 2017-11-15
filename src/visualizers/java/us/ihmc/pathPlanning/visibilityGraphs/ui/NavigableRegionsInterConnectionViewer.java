package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class NavigableRegionsInterConnectionViewer extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final MeshView connectionsMeshView = new MeshView();
   private final double connectionsLineThickness = 0.005;

   private final AtomicReference<Mesh> connectionsMeshToRender = new AtomicReference<>(null);
   private Mesh connectionsMeshRendered = null;
   private final AtomicReference<Boolean> resetRequested;
   private NavigableRegionsManager navigableRegionsManager;

   public NavigableRegionsInterConnectionViewer(REAMessager messager, NavigableRegionsManager navigableRegionsManager)
   {
      this.navigableRegionsManager = navigableRegionsManager;
      connectionsMeshView.setMouseTransparent(true);
      connectionsMeshView.setMaterial(new PhongMaterial(Color.CRIMSON));

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowInterConnections, this::handleShowThreadSafe);
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
         connectionsMeshView.setMesh(null);
      else
      {
         processInterConnections(navigableRegionsManager);

         connectionsMeshView.setMesh(connectionsMeshRendered);
      }
   }

   public void processInterConnections(NavigableRegionsManager navigableRegionsManager)
   {
      if (VERBOSE)
         PrintTools.info(this, "Building mesh for inter-connections.");
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      
      List<Connection> connections = navigableRegionsManager.getConnectionPoints();
      for (Connection connection : connections)
         meshBuilder.addLine(connection.getPoint1(), connection.getPoint2(), connectionsLineThickness);
      connectionsMeshToRender.set(meshBuilder.generateMesh());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         connectionsMeshView.setMesh(null);
         connectionsMeshToRender.getAndSet(null);
         return;
      }

      Mesh newMesh = connectionsMeshToRender.getAndSet(null);

      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering inter-connection lines.");
         connectionsMeshRendered = newMesh;
         connectionsMeshView.setMesh(newMesh);
      }
   }

   public Node getRoot()
   {
      return connectionsMeshView;
   }
}
