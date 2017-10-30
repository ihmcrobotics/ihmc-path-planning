package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.ThreadTools;

public class VisibilityGraphsRenderer
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicReference<NavigableRegionsManager> navigableRegionsManagerReference = new AtomicReference<>(null);
   private final AtomicReference<List<Point3D>> bodyPathReference = new AtomicReference<>(null);

   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final NavigableRegionInnerVizMapMeshViewer navigableRegionInnerVizMapMeshViewer;
   private final NavigableRegionsInterConnectionViewer navigableRegionsInterConnectionViewer;

   public VisibilityGraphsRenderer(REAMessager messager)
   {
      planarRegionsReference = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      startPositionReference = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goalPositionReference = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);

      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsComputePath, request -> computePathOnThread());

      bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      root.getChildren().add(bodyPathMeshViewer.getRoot());
      navigableRegionInnerVizMapMeshViewer = new NavigableRegionInnerVizMapMeshViewer(messager);
      root.getChildren().add(navigableRegionInnerVizMapMeshViewer.getRoot());
      navigableRegionsInterConnectionViewer = new NavigableRegionsInterConnectionViewer(messager);
      root.getChildren().add(navigableRegionsInterConnectionViewer.getRoot());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      startPositionReference.set(null);
      goalPositionReference.set(null);
   }

   public void start()
   {
      bodyPathMeshViewer.start();
      navigableRegionInnerVizMapMeshViewer.start();
      
   }

   public void stop()
   {
      bodyPathMeshViewer.stop();
      navigableRegionInnerVizMapMeshViewer.start();
   }

   private void computePathOnThread()
   {
      executorService.submit(this::computePath);
   }

   private void computePath()
   {
      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if (planarRegionsList == null)
         return;

      Point3D start = startPositionReference.get();

      if (start == null)
         return;

      Point3D goal = goalPositionReference.get();

      if (goal == null)
         return;

      if (VERBOSE)
         PrintTools.info(this, "Computing body path.");

      try
      {
         List<PlanarRegion> planarRegions = planarRegionsList.getPlanarRegionsAsList();
         planarRegions = planarRegions.stream().filter(region -> region.getConcaveHullSize() > 2).collect(Collectors.toList());
         NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(planarRegions);
         navigableRegionsManagerReference.set(navigableRegionsManager);

         List<Point3D> bodyPath = navigableRegionsManager.calculateBodyPath(start, goal);
         bodyPathReference.set(bodyPath);
         bodyPathMeshViewer.processBodyPath(bodyPath);
         navigableRegionInnerVizMapMeshViewer.processNavigableRegions(navigableRegionsManager.getListOfLocalPlanners());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
