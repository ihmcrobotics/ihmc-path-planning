package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoVisibilityGraphParameters implements VisibilityGraphsParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry("VisibilityGraphParameters");

   private final YoInteger numberOfForcedConnections = new YoInteger("numberOfForcedConnections", registry);
   private final YoDouble minimumConnectionDistanceForRegions = new YoDouble("minimumConnectionDistanceForRegions", registry);
   private final YoDouble normalZThresholdForAccessibleRegions = new YoDouble("normalZThresholdForAccessibleRegions", registry);
   private final YoDouble normalZThresholdForPolygonObstacles = new YoDouble("normalZThresholdForPolygonObstacles", registry);
   private final YoDouble extrusionDistance = new YoDouble("extrusionDistance", registry);
   private final YoDouble extrusionDistanceIfNotTooHighToStep = new YoDouble("extrusionDistanceIfNotTooHighToStep", registry);
   private final YoDouble tooHighToStepDistance = new YoDouble("tooHighToStepDistance", registry);
   private final YoDouble clusterResolution = new YoDouble("clusterResolution", registry);
   private final YoDouble explorationDistance = new YoDouble("explorationDistance", registry);

   public YoVisibilityGraphParameters(VisibilityGraphsParameters defaults, YoVariableRegistry parentRegistry)
   {
      this.numberOfForcedConnections.set(defaults.getNumberOfForcedConnections());
      this.minimumConnectionDistanceForRegions.set(defaults.getMinimumConnectionDistanceForRegions());
      this.normalZThresholdForAccessibleRegions.set(defaults.getNormalZThresholdForAccessibleRegions());
      this.normalZThresholdForPolygonObstacles.set(defaults.getNormalZThresholdForPolygonObstacles());
      this.extrusionDistance.set(defaults.getExtrusionDistance());
      this.extrusionDistanceIfNotTooHighToStep.set(defaults.getExtrusionDistanceIfNotTooHighToStep());
      this.tooHighToStepDistance.set(defaults.getTooHighToStepDistance());
      this.clusterResolution.set(defaults.getClusterResolution());
      this.explorationDistance.set(defaults.getExplorationDistanceFromStartGoal());

      parentRegistry.addChild(registry);
   }

   @Override
   public int getNumberOfForcedConnections()
   {
      return numberOfForcedConnections.getIntegerValue();
   }

   @Override
   public double getMinimumConnectionDistanceForRegions()
   {
      return minimumConnectionDistanceForRegions.getDoubleValue();
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normalZThresholdForAccessibleRegions.getDoubleValue();
   }

   @Override
   public double getNormalZThresholdForPolygonObstacles()
   {
      return normalZThresholdForPolygonObstacles.getDoubleValue();
   }

   @Override
   public double getExtrusionDistance()
   {
      return extrusionDistance.getDoubleValue();
   }

   @Override
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return extrusionDistanceIfNotTooHighToStep.getDoubleValue();
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return tooHighToStepDistance.getDoubleValue();
   }

   @Override
   public double getClusterResolution()
   {
      return clusterResolution.getDoubleValue();
   }

   @Override
   public double getExplorationDistanceFromStartGoal()
   {
      return explorationDistance.getDoubleValue();
   }
}
