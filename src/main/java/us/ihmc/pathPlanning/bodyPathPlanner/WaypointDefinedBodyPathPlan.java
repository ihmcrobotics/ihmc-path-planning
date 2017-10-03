package us.ihmc.pathPlanning.bodyPathPlanner;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class WaypointDefinedBodyPathPlan implements BodyPathPlanner
{
   private static final double epsilon = 1e-8;

   private final List<FramePoint3D> waypoints = new ArrayList<>();

   private double[] maxAlphas;
   private double[] segmentLengths;

   public void setWaypoints(List<FramePoint3D> waypoints)
   {
      this.waypoints.addAll(waypoints);
      this.maxAlphas = new double[waypoints.size() - 1];
      this.segmentLengths = new double[waypoints.size() - 1];
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public void compute(FramePoint3D startPoint, FramePoint3D goalPoint)
   {
      double totalPathLength = 0.0;

      for (int i = 0; i < segmentLengths.length; i++)
      {
         FramePoint3D segmentStart = waypoints.get(i);
         FramePoint3D segmentEnd = waypoints.get(i);
         segmentLengths[i] = segmentEnd.distance(segmentStart);
         totalPathLength = totalPathLength + segmentLengths[i];
      }

      for (int i = 0; i < segmentLengths.length; i++)
      {
         double previousMaxAlpha = (i == 0) ? 0 : maxAlphas[i - 1];
         maxAlphas[i] = previousMaxAlpha + segmentLengths[i] / totalPathLength;
      }
   }

   @Override
   public void getPointAlongPath(double alpha, FramePoint3D pointToPack)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      FramePoint3D firstPoint = waypoints.get(segmentIndex);
      FramePoint3D secondPoint = waypoints.get(segmentIndex + 1);

      double percentageAlongSegment = segmentIndex == 0 ? alpha : alpha - maxAlphas[segmentIndex - 1];
      FrameVector3D offsetVector = new FrameVector3D(secondPoint);
      offsetVector.sub(firstPoint);
      offsetVector.scale(percentageAlongSegment);

      pointToPack.setIncludingFrame(firstPoint);
      pointToPack.add(offsetVector);
   }

   @Override
   public double getClosestPoint(FramePoint3D point, FramePoint3D pointToPack)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());

      double closestPointDistance = Double.POSITIVE_INFINITY;
      FramePoint3D tempClosestPoint = new FramePoint3D();

      for (int i = 0; i < segmentLengths.length; i++)
      {
         FramePoint3D segmentStart = waypoints.get(i);
         FramePoint3D segmentEnd = waypoints.get(i + 1);

         EuclidFrameTools.orthogonalProjectionOnLineSegment3D(point, segmentStart, segmentEnd, tempClosestPoint);
         double distance = tempClosestPoint.distance(point);
         if(distance < closestPointDistance)
         {
            closestPointDistance = distance;
            pointToPack.setIncludingFrame(tempClosestPoint);
         }
      }

      return closestPointDistance;
   }

   @Override
   public double computePathLength(double alpha)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      double segmentLength = (maxAlphas[segmentIndex] - alpha) * segmentLengths[segmentIndex];

      for (int i = segmentIndex + 1; i < segmentLengths.length; i++)
      {
         segmentLength = segmentLength + segmentLengths[i];
      }

      return segmentLength;
   }

   private int getRegionIndexFromAlpha(double alpha)
   {
      for (int i = 0; i < maxAlphas.length; i++)
      {
         if(maxAlphas[i] >= alpha + epsilon)
            return i;
      }

      throw new RuntimeException("Alpha must be between [0,1] and maxAlphas highest value must be 1.0");
   }
}
