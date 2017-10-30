package us.ihmc.pathPlanning.visibilityGraphs.ui;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowBodyPath;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowInterConnections;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowLocalGraphs;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.ShowPlanarRegions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.VisibilityGraphsComputePath;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;

public class VisibilityGraphsAnchorPaneController
{
   @FXML
   private ToggleButton showBodyPathToggleButton;
   @FXML
   private ToggleButton showLocalGraphsToggleButton;
   @FXML
   private ToggleButton showInterConnectionsToggleButton;
   @FXML
   private ToggleButton showPlanarRegionsToggleButton;

   private SimpleUIMessager messager;

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowLocalGraphs, showLocalGraphsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterConnections, showInterConnectionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);
   }

   @FXML
   public void computePath()
   {
      messager.submitMessage(VisibilityGraphsComputePath, true);
   }
}
