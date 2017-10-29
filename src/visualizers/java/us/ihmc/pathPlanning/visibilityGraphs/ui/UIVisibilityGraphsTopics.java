package us.ihmc.pathPlanning.visibilityGraphs.ui;

import us.ihmc.robotEnvironmentAwareness.communication.APIFactory;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class UIVisibilityGraphsTopics
{
   private static final APIFactory apiFactory = new APIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");

   private static final TypedTopicTheme<Boolean> Load = apiFactory.createTypedTopicTheme("Load");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.getRootCategory(apiFactory.createCategoryTheme("VizGraphs"));

   public static final Topic<Boolean> LoadPlanarRegion = Root.child(PlanarRegion).topic(Load);
   public static final Topic<PlanarRegionsList> PlanarRegionData = Root.child(PlanarRegion).topic(Data);

   public static final API API = apiFactory.getAPIAndCloseFactory();
}
