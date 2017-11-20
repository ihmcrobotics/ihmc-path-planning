package us.ihmc.pathPlanning.visibilityGraphs;

public class DefaultVisibilityGraphParameters implements VisibilityGraphsParameters
{
   @Override
   public int getNumberOfForcedConnections()
   {
      return 5;
   }

   @Override
   public double getMinimumConnectionDistanceForRegions()
   {
      return 0.001;
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return 0.8;
   }

   @Override
   public double getNormalZThresholdForPolygonObstacles()
   {
      return 0.8;
   }

   @Override
   public double getExtrusionDistance()
   {
      return 0.8;
   }

   @Override
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return 0.25;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return 0.5;
   }

   @Override
   public double getClusterResolution()
   {
      return 0.5;
   }
}
