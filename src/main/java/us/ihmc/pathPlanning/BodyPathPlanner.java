package us.ihmc.pathPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BodyPathPlanner
{
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   /**
    * Computes a new body path plan given a start and goal point
    */
   public void compute(FramePoint3D startPoint, FramePoint3D goalPoint);

   /**
    * Computes and packs a point along the curve specified by
    * alpha, which goes from 0.0 (start) to 1.0 (goal)
    */
   public void getPointAlongPath(double alpha, FramePoint3D pointToPack);

   /**
    * Computes the point along the curve closest to the given point
    * @return alpha corresponding to the packed point
    */
   public double getClosestPoint(FramePoint3D point, FramePoint3D pointToPack);

   /**
    * Returns arc length of the body path from alpha to the goal point
    */
   public double computePathLength(double alpha);
}