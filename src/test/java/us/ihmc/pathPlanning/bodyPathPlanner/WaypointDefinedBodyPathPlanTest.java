package us.ihmc.pathPlanning.bodyPathPlanner;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JCheckBox;
import javax.swing.JFrame;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WaypointDefinedBodyPathPlanTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean showPlotter = false;
   private static final double epsilon = 1.0e-15;

   @Rule
   public TestName name = new TestName();

   @Test
   public void testSimpleBodyPath()
   {
      WaypointDefinedBodyPathPlan plan = new WaypointDefinedBodyPathPlan();
      List<FramePoint3D> waypoints = new ArrayList<FramePoint3D>();
      waypoints.add(new FramePoint3D(worldFrame, 0.0, 0.0, 0.0));
      waypoints.add(new FramePoint3D(worldFrame, 0.5, 0.0, 0.0));
      waypoints.add(new FramePoint3D(worldFrame, 1.0, 1.0, 0.0));
      plan.setWaypoints(waypoints);
      plan.compute(null, null);

      // test path length method
      double segmentLength1 = 0.5;
      double segmentLength2 = Math.sqrt(0.5 * 0.5 + 1.0 * 1.0);
      double toalLength = segmentLength1 + segmentLength2;
      Assert.assertEquals(toalLength, plan.computePathLength(0.0), epsilon);
      Assert.assertEquals(segmentLength1 / 2.0 + segmentLength2, plan.computePathLength(0.5 * segmentLength1 / toalLength), epsilon);
      Assert.assertEquals(segmentLength2, plan.computePathLength(segmentLength1 / toalLength), epsilon);
      Assert.assertEquals(segmentLength2 / 2.0, plan.computePathLength(1.0 - 0.5 * segmentLength2 / toalLength), epsilon);
      Assert.assertEquals(0.0, plan.computePathLength(1.0), epsilon);

      // test point along path method
      FramePoint3D testPoint = new FramePoint3D();
      plan.getPointAlongPath(0.0, testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(0), testPoint, epsilon);
      plan.getPointAlongPath(segmentLength1 / toalLength, testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(1), testPoint, epsilon);
      plan.getPointAlongPath(1.0, testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(2), testPoint, epsilon);

      // test get closest point method
      double d1 = plan.getClosestPoint(new FramePoint3D(worldFrame, -1.0, 0.0, 0.0), testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(0), testPoint, epsilon);
      Assert.assertEquals(0.0, d1, epsilon);
      double d2 = plan.getClosestPoint(new FramePoint3D(worldFrame, 10.0, 0.0, 0.0), testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(2), testPoint, epsilon);
      Assert.assertEquals(1.0, d2, epsilon);
      double d3 = plan.getClosestPoint(new FramePoint3D(worldFrame, 10.0, -10.0, 0.0), testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(waypoints.get(1), testPoint, epsilon);
      Assert.assertEquals(segmentLength1 / toalLength, d3, epsilon);
      double d4 = plan.getClosestPoint(new FramePoint3D(worldFrame, 0.25, 0.1, 0.0), testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(new FramePoint3D(worldFrame, 0.25, 0.0, 0.0), testPoint, epsilon);
      Assert.assertEquals(0.5 * segmentLength1 / toalLength, d4, epsilon);
      double d5 = plan.getClosestPoint(new FramePoint3D(worldFrame, 0.75 + 1.0, 0.5 - 0.5, 0.0), testPoint);
      EuclidCoreTestTools.assertTuple3DEquals(new FramePoint3D(worldFrame, 0.75, 0.5, 0.0), testPoint, epsilon);
      Assert.assertEquals(1.0 - 0.5 * segmentLength2 / toalLength, d5, epsilon);

      if (showPlotter)
      {
         showPlotter(plan, name.getMethodName());
      }
   }

   public static void showPlotter(WaypointDefinedBodyPathPlan plan, String testName)
   {
      int markers = 100;
      YoVariableRegistry registry = new YoVariableRegistry(testName);
      YoGraphicsListRegistry graphicsList = new YoGraphicsListRegistry();
      for (int i = 0; i < markers; i++)
      {
         double alpha = (double) i / (double) (markers - 1);
         FramePoint3D point = new FramePoint3D();
         plan.getPointAlongPath(alpha, point);
         YoFramePoint yoPoint = new YoFramePoint("Point" + i, worldFrame, registry);
         yoPoint.set(point);
         YoGraphicPosition pointVis = new YoGraphicPosition("Point" + i, yoPoint, 0.01, YoAppearance.Blue());
         graphicsList.registerArtifact(testName, pointVis.createArtifact());
      }

      showPlotter(graphicsList, testName);
   }

   public static void showPlotter(YoGraphicsListRegistry yoGraphicsListRegistry, String windowName)
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(2.0);

      ArrayList<ArtifactList> artifactLists = new ArrayList<>();
      yoGraphicsListRegistry.getRegisteredArtifactLists(artifactLists);
      for (ArtifactList artifactList : artifactLists)
      {
         artifactList.setVisible(true);
      }

      JFrame frame = new JFrame(windowName);
      Dimension preferredSize = new Dimension(600, 600);
      frame.setPreferredSize(preferredSize);

      JCheckBox doneBox = new JCheckBox("Done");
      PlotterShowHideMenu plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);

      frame.add(doneBox, BorderLayout.SOUTH);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);

      frame.setSize(preferredSize);
      frame.setVisible(true);

      yoGraphicsListRegistry.addArtifactListsToPlotter(plotter);

      while (!doneBox.isSelected())
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

      frame.setVisible(false);
      frame.dispose();
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(WaypointDefinedBodyPathPlan.class, WaypointDefinedBodyPathPlanTest.class);
   }
}
