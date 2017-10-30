package us.ihmc.pathPlanning.visibilityGraphs.ui;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowBodyPath;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowLocalGraphs;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;

public class VisibilityGraphsAnchorPaneController
{
   @FXML
   private ToggleButton showBodyPathToggleButton;
   @FXML
   private ToggleButton showLocalGraphsToggleButton;

   private SimpleUIMessager messager;

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowLocalGraphs, showLocalGraphsToggleButton.selectedProperty(), true);
   }

   @FXML
   public void computePath()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.VisibilityGraphsComputePath, true);
   }
}
