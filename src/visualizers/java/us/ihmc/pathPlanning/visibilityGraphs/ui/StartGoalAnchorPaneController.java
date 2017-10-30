package us.ihmc.pathPlanning.visibilityGraphs.ui;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.GoalEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.StartEditModeEnabled;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javafx.beans.Observable;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;

public class StartGoalAnchorPaneController
{
   @FXML
   private ToggleButton placeStartToggleButton;
   @FXML
   private ToggleButton placeGoalToggleButton;
   @FXML
   private TextField startXTextField;
   @FXML
   private TextField startYTextField;
   @FXML
   private TextField startZTextField;
   @FXML
   private TextField goalXTextField;
   @FXML
   private TextField goalYTextField;
   @FXML
   private TextField goalZTextField;

   private SimpleUIMessager messager;
   private final SimpleObjectProperty<Point3D> startPositionProperty = new SimpleObjectProperty<>(this, "startPositionProperty",
                                                                                                  new Point3D(Double.NaN, Double.NaN, Double.NaN));
   private final SimpleObjectProperty<Point3D> goalPositionProperty = new SimpleObjectProperty<>(this, "goalPositionProperty",
                                                                                                 new Point3D(Double.NaN, Double.NaN, Double.NaN));

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(StartEditModeEnabled, placeStartToggleButton.selectedProperty(), false);
      messager.bindBidirectional(StartEditModeEnabled, placeGoalToggleButton.disableProperty(), false);

      messager.bindBidirectional(GoalEditModeEnabled, placeGoalToggleButton.selectedProperty(), false);
      messager.bindBidirectional(GoalEditModeEnabled, placeStartToggleButton.disableProperty(), false);

      messager.bindPropertyToTopic(UIVisibilityGraphsTopics.StartPosition, startPositionProperty);
      messager.bindPropertyToTopic(UIVisibilityGraphsTopics.GoalPosition, goalPositionProperty);
      NumberFormat formatter = new DecimalFormat("0.000;-0.000");
      startPositionProperty.addListener((Observable o) -> startXTextField.setText(formatter.format(startPositionProperty.getValue().getX())));
      startPositionProperty.addListener((Observable o) -> startYTextField.setText(formatter.format(startPositionProperty.getValue().getY())));
      startPositionProperty.addListener((Observable o) -> startZTextField.setText(formatter.format(startPositionProperty.getValue().getZ())));

      goalPositionProperty.addListener((Observable o) -> goalXTextField.setText(formatter.format(goalPositionProperty.getValue().getX())));
      goalPositionProperty.addListener((Observable o) -> goalYTextField.setText(formatter.format(goalPositionProperty.getValue().getY())));
      goalPositionProperty.addListener((Observable o) -> goalZTextField.setText(formatter.format(goalPositionProperty.getValue().getZ())));

      messager.registerTopicListener(UIVisibilityGraphsTopics.GlobalReset, reset -> clearStartGoalTextFields());
   }

   private void clearStartGoalTextFields()
   {
      startXTextField.clear();
      startYTextField.clear();
      startZTextField.clear();

      goalXTextField.clear();
      goalYTextField.clear();
      goalZTextField.clear();
   }
}
