package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.fxml.FXML;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleUIMenuController
{
   private static final boolean VERBOSE = true;

   private REAMessager messager;
   private Window ownerWindow;

   public void attachMessager(REAMessager messager)
   {
      this.messager = messager;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   @FXML
   public void loadPlanarRegion()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.LoadPlanarRegion, true);
      PlanarRegionsList loadedPlanarRegions = PlanarRegionDataImporter.importUsingFileChooser(ownerWindow);
      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Loaded planar regions, broadcasting data.");
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
      }
      else
      {
         if (VERBOSE)
            PrintTools.info(this, "Failed to load planar regions.");
      }
   }
}
