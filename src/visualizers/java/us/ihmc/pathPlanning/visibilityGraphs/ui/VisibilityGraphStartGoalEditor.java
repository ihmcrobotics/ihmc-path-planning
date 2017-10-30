package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class VisibilityGraphStartGoalEditor extends AnimationTimer
{
   private static final boolean VERBOSE = true;
   private static final double RADIUS = 0.05;

   private final Group root = new Group();

   private boolean isStartRendered = false;
   private final Sphere startSphere = new Sphere(RADIUS);
   private boolean isGoalRendered = false;
   private final Sphere goalSphere = new Sphere(RADIUS);

   private final PhongMaterial startOpaqueMaterial = new PhongMaterial(Color.GREEN);
   private final PhongMaterial startTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));
   private final PhongMaterial goalOpaqueMaterial = new PhongMaterial(Color.RED);
   private final PhongMaterial goalTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private boolean isRayCastInterceptorAttached = false;
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);

   private final EventHandler<MouseEvent> leftClickInterceptor;
   private boolean isLeftClickInterceptorAttached = false;
   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   private final REAMessager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

   public VisibilityGraphStartGoalEditor(REAMessager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      startSphere.setMouseTransparent(true);
      goalSphere.setMouseTransparent(true);

      startEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.StartEditModeEnabled, false);
      goalEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.GoalEditModeEnabled, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.GlobalReset, reset -> clearStartAndGoal());

      rayCastInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            PickResult pickResult = event.getPickResult();
            Node intersectedNode = pickResult.getIntersectedNode();
            if (intersectedNode == null || intersectedNode instanceof SubScene)
               return;
            javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
            javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

            Point3D interception = new Point3D();
            interception.setX(scenePoint.getX());
            interception.setY(scenePoint.getY());
            interception.setZ(scenePoint.getZ());

            latestInterception.set(interception);
         }
      };

      leftClickInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != MouseButton.PRIMARY)
               return;

            if (event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
               positionValidated.set(true);
         }
      };
   }

   public void clearStartAndGoal()
   {
      if (!root.getChildren().isEmpty())
      {
         root.getChildren().clear();
         isStartRendered = false;
         isGoalRendered = false;
      }
   }

   @Override
   public void handle(long now)
   {
      if (startEditModeEnabled.get() && goalEditModeEnabled.get())
         throw new RuntimeException("Cannot edit start AND goal together.");

      if (startEditModeEnabled.get() || goalEditModeEnabled.get())
      {
         attachEvenHandlers();
      }
      else
      {
         removeEventHandlers();
         return;
      }

      if (startEditModeEnabled.get())
      {
         startSphere.setMaterial(startTransparentMaterial);

         if (!isStartRendered)
         {
            if (VERBOSE)
               PrintTools.info(this, "Adding start sphere to scene.");
            root.getChildren().add(startSphere);
            isStartRendered = true;
         }

         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            setStartPosition(interception);
         }

         if (positionValidated.getAndSet(false))
         {
            if (VERBOSE)
               PrintTools.info(this, "Start position is validated: " + getGoalPosition());
            startSphere.setMaterial(startOpaqueMaterial);
            messager.submitMessage(UIVisibilityGraphsTopics.StartEditModeEnabled, false);
            messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, getStartPosition());
         }
         return;
      }

      if (goalEditModeEnabled.get())
      {
         goalSphere.setMaterial(goalTransparentMaterial);

         if (!isGoalRendered)
         {
            if (VERBOSE)
               PrintTools.info(this, "Adding goal sphere to scene.");
            root.getChildren().add(goalSphere);
            isGoalRendered = true;
         }

         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            setGoalPosition(interception);
         }

         if (positionValidated.getAndSet(false))
         {
            if (VERBOSE)
               PrintTools.info(this, "Goal position is validated: " + getGoalPosition());
            goalSphere.setMaterial(goalOpaqueMaterial);
            messager.submitMessage(UIVisibilityGraphsTopics.GoalEditModeEnabled, false);
            messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, getGoalPosition());
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching ray cast event handler.");
         sceneNode.addEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = true;
      }
      if (!isLeftClickInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching left click event handler.");
         sceneNode.addEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = true;
      }
   }

   private void removeEventHandlers()
   {
      if (isRayCastInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = false;
      }
      if (isLeftClickInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = false;
      }
   }

   private void setStartPosition(Point3D position)
   {
      startSphere.setTranslateX(position.getX());
      startSphere.setTranslateY(position.getY());
      startSphere.setTranslateZ(position.getZ());
   }

   private Point3D getStartPosition()
   {
      if (!isStartRendered)
         return null;
      else
         return new Point3D(startSphere.getTranslateX(), startSphere.getTranslateY(), startSphere.getTranslateZ());
   }

   private void setGoalPosition(Point3D position)
   {
      goalSphere.setTranslateX(position.getX());
      goalSphere.setTranslateY(position.getY());
      goalSphere.setTranslateZ(position.getZ());
   }

   private Point3D getGoalPosition()
   {
      if (!isGoalRendered)
         return null;
      else
         return new Point3D(goalSphere.getTranslateX(), goalSphere.getTranslateY(), goalSphere.getTranslateZ());
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
