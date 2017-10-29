package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerSharedVariables;

public class SimpleVisibilityGraphsUI
{
   private final REAMessagerSharedVariables messager = new REAMessagerSharedVariables(UIVisibilityGraphsTopics.API);
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final VizGraphsPlanarRegionViewer planarRegionViewer;

   @FXML
   private SimpleUIMenuController simpleUIMenuController;

   public SimpleVisibilityGraphsUI(Stage primaryStage) throws IOException
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      messager.startMessager();

      planarRegionViewer = new VizGraphsPlanarRegionViewer(messager);
      planarRegionViewer.start();

      simpleUIMenuController.attachMessager(messager);
      simpleUIMenuController.setMainWindow(primaryStage);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws IOException
   {
      primaryStage.show();
   }

   public void stop()
   {
      messager.closeMessager();
      planarRegionViewer.stop();
   }
}
