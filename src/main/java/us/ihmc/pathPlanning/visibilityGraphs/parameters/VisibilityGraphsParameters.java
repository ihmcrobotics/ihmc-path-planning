package us.ihmc.pathPlanning.visibilityGraphs.parameters;

public class VisibilityGraphsParameters
{
   /**
    * Specifies the distance to shrink the concave hull of each navigable region.
    */
   private double regionHullExtrusionDistance = 0.01;
   /**
    * Determines the height threshold that separates a small from a tall obstacle.
    */
   private double obstacleHeightThreshold = 0.5;
   /**
    * Specifies the extrusion distance to perform on the projection of a tall obstacle onto a
    * navigable region.
    */
   private double tallObstacleExtrusionDistance = 0.8;
   /**
    * Specifies the extrusion distance to perform on the projection of a small obstacle onto a
    * navigable region.
    */
   private double smallObstacleExtrusionDistance = 0.1;
   /**
    * Defines the maximum length that a connection between two navigable regions can be.
    */
   private double maxInterRegionConnectionLength = 0.0;

   public VisibilityGraphsParameters()
   {
   }

   public double getRegionHullExtrusionDistance()
   {
      return regionHullExtrusionDistance;
   }

   public double getObstacleHeightThreshold()
   {
      return obstacleHeightThreshold;
   }

   public double getTallObstacleExtrusionDistance()
   {
      return tallObstacleExtrusionDistance;
   }

   public double getSmallObstacleExtrusionDistance()
   {
      return smallObstacleExtrusionDistance;
   }

   public double getMaxInterRegionConnectionLength()
   {
      return maxInterRegionConnectionLength;
   }

   public void setRegionHullExtrusionDistance(double regionHullExtrusionDistance)
   {
      this.regionHullExtrusionDistance = regionHullExtrusionDistance;
   }

   public void setObstacleHeightThreshold(double obstacleHeightThreshold)
   {
      this.obstacleHeightThreshold = obstacleHeightThreshold;
   }

   public void setTallObstacleExtrusionDistance(double tallObstacleExtrusionDistance)
   {
      this.tallObstacleExtrusionDistance = tallObstacleExtrusionDistance;
   }

   public void setSmallObstacleExtrusionDistance(double smallObstacleExtrusionDistance)
   {
      this.smallObstacleExtrusionDistance = smallObstacleExtrusionDistance;
   }

   public void setMaxInterRegionConnectionLength(double maxInterRegionConnectionLength)
   {
      this.maxInterRegionConnectionLength = maxInterRegionConnectionLength;
   }
}
