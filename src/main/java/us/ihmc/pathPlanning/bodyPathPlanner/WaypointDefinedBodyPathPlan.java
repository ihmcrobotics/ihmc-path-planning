package us.ihmc.pathPlanning.bodyPathPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class WaypointDefinedBodyPathPlan implements BodyPathPlanner
{
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
         FramePoint3D segmentEnd = waypoints.get(i + 1);
         segmentLengths[i] = segmentEnd.distance(segmentStart);
         totalPathLength = totalPathLength + segmentLengths[i];
      }

      for (int i = 0; i < segmentLengths.length; i++)
      {
         double previousMaxAlpha = (i == 0) ? 0.0 : maxAlphas[i - 1];
         maxAlphas[i] = previousMaxAlpha + segmentLengths[i] / totalPathLength;
      }
   }

   @Override
   public void getPointAlongPath(double alpha, FramePoint3D pointToPack)
   {
      int segmentIndex = getRegionIndexFromAlpha(alpha);
      FramePoint3D firstPoint = waypoints.get(segmentIndex);
      FramePoint3D secondPoint = waypoints.get(segmentIndex + 1);

      double alphaInSegment = getPercentInSegment(segmentIndex, alpha);

      pointToPack.interpolate(firstPoint, secondPoint, alphaInSegment);
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
         if (distance < closestPointDistance)
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
      double alphaInSegment = getPercentInSegment(segmentIndex, alpha);

      double segmentLength = (1.0 - alphaInSegment) * segmentLengths[segmentIndex];
      for (int i = segmentIndex + 1; i < segmentLengths.length; i++)
      {
         segmentLength = segmentLength + segmentLengths[i];
      }

      return segmentLength;
   }

   private double getPercentInSegment(int segment, double alpha)
   {
      boolean firstSegment = segment == 0;
      double alphaSegmentStart = firstSegment ? 0.0 : maxAlphas[segment - 1];
      double alphaSegmentEnd = maxAlphas[segment];
      return (alpha - alphaSegmentStart) / (alphaSegmentEnd - alphaSegmentStart);
   }

   private int getRegionIndexFromAlpha(double alpha)
   {
      for (int i = 0; i < maxAlphas.length; i++)
      {
         if (maxAlphas[i] >= alpha)
            return i;
      }

      throw new RuntimeException("Alpha = " + alpha + "\nalpha must be between [0,1] and maxAlphas highest value must be 1.0.");
   }
}
