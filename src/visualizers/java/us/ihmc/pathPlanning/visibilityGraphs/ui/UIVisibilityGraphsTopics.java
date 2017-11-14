package us.ihmc.pathPlanning.visibilityGraphs.ui;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class UIVisibilityGraphsTopics
{
   private static final APIFactory apiFactory = new APIFactory();

   private static final CategoryTheme VisibilityGraphs = apiFactory.createCategoryTheme("VisibilityGraphs");
   private static final CategoryTheme LocalGraphs = apiFactory.createCategoryTheme("LocalGraphs");
   private static final CategoryTheme Connections = apiFactory.createCategoryTheme("Connections");
   private static final CategoryTheme BodyPath = apiFactory.createCategoryTheme("BodyPath");
   private static final CategoryTheme Cluster = apiFactory.createCategoryTheme("Cluster");
   private static final CategoryTheme RawPoints = apiFactory.createCategoryTheme("RawPoints");
   private static final CategoryTheme NavigableExtrusions = apiFactory.createCategoryTheme("NavigableExtrusions");
   private static final CategoryTheme NonNavigableExtrusions = apiFactory.createCategoryTheme("NonNavigableExtrusions");
   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme UnitTest = apiFactory.createCategoryTheme("UnitTest");

   
   private static final TypedTopicTheme<Boolean> Load = apiFactory.createTypedTopicTheme("Load");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Close = apiFactory.createTypedTopicTheme("Close");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Boolean> Export = apiFactory.createTypedTopicTheme("Export");
   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.getRootCategory(apiFactory.createCategoryTheme("VizGraphs"));

   public static final Topic<Boolean> LoadPlanarRegion = Root.child(PlanarRegion).topic(Load);
   public static final Topic<PlanarRegionsList> PlanarRegionData = Root.child(PlanarRegion).topic(Data);

   public static final Topic<Boolean> StartEditModeEnabled = Root.child(Start).child(EditMode).topic(Enable);
   public static final Topic<Boolean> GoalEditModeEnabled = Root.child(Goal).child(EditMode).topic(Enable);
   public static final Topic<Point3D> StartPosition = Root.child(Start).topic(Position);
   public static final Topic<Point3D> GoalPosition = Root.child(Goal).topic(Position);

   public static final Topic<Boolean> VisibilityGraphsComputePath = Root.child(VisibilityGraphs).topic(ComputePath);
   public static final Topic<Boolean> ShowBodyPath = Root.child(VisibilityGraphs).child(BodyPath).topic(Show);
   public static final Topic<Boolean> ShowLocalGraphs = Root.child(VisibilityGraphs).child(LocalGraphs).topic(Show);
   public static final Topic<Boolean> ShowInterConnections = Root.child(VisibilityGraphs).child(Connections).topic(Show);
   public static final Topic<Boolean> ShowPlanarRegions = Root.child(PlanarRegion).topic(Show);
   public static final Topic<Boolean> ShowClusterRawPoints = Root.child(VisibilityGraphs).child(Cluster).child(RawPoints).topic(Show);
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NavigableExtrusions).topic(Show);
   public static final Topic<Boolean> CloseClusterNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NavigableExtrusions).topic(Close);
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NonNavigableExtrusions).topic(Show);
   public static final Topic<Boolean> CloseClusterNonNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NonNavigableExtrusions)
                                                                               .topic(Close);
   public static final Topic<Boolean> exportUnitTestDataFile = Root.child(UnitTest).topic(Export);
   public static final Topic<String> exportUnitTestPath = Root.child(UnitTest).topic(Path);

   public static final Topic<Boolean> GlobalReset = Root.topic(Reset);

   public static final API API = apiFactory.getAPIAndCloseFactory();
}
