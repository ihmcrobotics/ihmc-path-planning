package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class VisibilityGraphStartGoalViewer extends AnimationTimer
{
   private static final double RADIUS = 0.05;

   private final Group root = new Group();

   private final Sphere startSphere = new Sphere(RADIUS);
   private final Sphere goalSphere = new Sphere(RADIUS);

   public enum StartGoalViewMode
   {
      TRANSLUCENT, OPAQUE
   };

   private final PhongMaterial startOpaqueMaterial = new PhongMaterial(Color.GREEN);
   private final PhongMaterial startTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));
   private final PhongMaterial goalOpaqueMaterial = new PhongMaterial(Color.RED);
   private final PhongMaterial goalTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   public VisibilityGraphStartGoalViewer(REAMessager messager)
   {
      startSphere.setMouseTransparent(true);
      goalSphere.setMouseTransparent(true);

      root.getChildren().add(startSphere);
      root.getChildren().add(goalSphere);

      startEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.StartEditModeEnabled, false);
      goalEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.GoalEditModeEnabled, false);
      startPositionReference = messager.createInput(UIVisibilityGraphsTopics.StartPosition, new Point3D());
      goalPositionReference = messager.createInput(UIVisibilityGraphsTopics.GoalPosition, new Point3D());
   }

   @Override
   public void handle(long now)
   {
      if (startEditModeEnabled.get())
         startSphere.setMaterial(startTransparentMaterial);
      else
         startSphere.setMaterial(startOpaqueMaterial);

      Point3D startPosition = startPositionReference.get();
      if (startPosition != null)
      {
         setStartPosition(startPosition);
      }

      if (goalEditModeEnabled.get())
         goalSphere.setMaterial(goalTransparentMaterial);
      else
         goalSphere.setMaterial(goalOpaqueMaterial);

      Point3D goalPosition = goalPositionReference.get();
      if (goalPosition != null)
      {
         setGoalPosition(goalPosition);
      }
   }

   private void setStartPosition(Point3D position)
   {
      startSphere.setTranslateX(position.getX());
      startSphere.setTranslateY(position.getY());
      startSphere.setTranslateZ(position.getZ());
   }

   private void setGoalPosition(Point3D position)
   {
      goalSphere.setTranslateX(position.getX());
      goalSphere.setTranslateY(position.getY());
      goalSphere.setTranslateZ(position.getZ());
   }

   private static Color toTransparentColor(Color opaqueColor, double opacity)
   {
      double red = opaqueColor.getRed();
      double green = opaqueColor.getGreen();
      double blue = opaqueColor.getBlue();
      return new Color(red, green, blue, opacity);
   }

   public Node getRoot()
   {
      return root;
   }
}
