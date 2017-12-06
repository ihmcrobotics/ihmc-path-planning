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
      return 0.55;
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
      return 0.1;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return 0.6;
   }

   @Override
   public double getClusterResolution()
   {
      return 0.2;
   }

   @Override
   public double getPlanarRegionMinArea()
   {
      return 0.05;
   }

   @Override
   public int getPlanarRegionMinSize()
   {
      return 5;
   }
}
