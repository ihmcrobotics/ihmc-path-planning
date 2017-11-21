package us.ihmc.pathPlanning.visibilityGraphs;

public interface VisibilityGraphsParameters
{
   public int getNumberOfForcedConnections();

   public double getMinimumConnectionDistanceForRegions();

   public double getNormalZThresholdForAccessibleRegions();

   public double getNormalZThresholdForPolygonObstacles();

   public double getExtrusionDistance();

   public double getExtrusionDistanceIfNotTooHighToStep();

   public double getTooHighToStepDistance();

   public double getClusterResolution();
}
