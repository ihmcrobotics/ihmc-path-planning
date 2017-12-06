package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ClusterManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.LinearRegression3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegionLocalPlanner
{
   private static final boolean debug = false;

   private final List<Cluster> clusters = new ArrayList<>();
   private final List<PlanarRegion> regions = new ArrayList<>();
   private List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
   private final List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
   private final List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
   private ReferenceFrame localReferenceFrame;

   private PlanarRegion homeRegion;

   private VisibilityMap localVisibilityMap;

   private ClusterManager clusterMgr;
   private VisibilityGraphsParameters visibilityGraphsParameters;

   public NavigableRegionLocalPlanner(List<PlanarRegion> regions, PlanarRegion homeRegion, VisibilityGraphsParameters visibilityGraphsParameters)
   {
      this.regions.addAll(regions);

      this.homeRegion = homeRegion;

      this.visibilityGraphsParameters = visibilityGraphsParameters;

      createLocalReferenceFrame();
   }

   private void createLocalReferenceFrame()
   {
      localReferenceFrame = new ReferenceFrame("regionLocalFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            homeRegion.getTransformToWorld(transformToParent);
         }
      };

      localReferenceFrame.update();
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
   }

   public ReferenceFrame getLocalReferenceFrame()
   {
      return localReferenceFrame;
   }

   public void processRegion()
   {
      regionsInsideHomeRegion = PlanarRegionTools.determineWhichRegionsAreInside(homeRegion, regions);

      //      System.out.println("Regions that are inside: " + regionsInsideHomeRegion.size());

      if (debug)
      {
         System.out.println(regionsInsideHomeRegion.size());
      }

      ClusterTools.classifyExtrusions(regionsInsideHomeRegion, homeRegion, lineObstacleRegions, polygonObstacleRegions,
                                      visibilityGraphsParameters.getNormalZThresholdForPolygonObstacles());
      regionsInsideHomeRegion = PlanarRegionTools.filterRegionsThatAreAboveHomeRegion(regionsInsideHomeRegion, homeRegion);

      ClusterTools.createClustersFromRegions(homeRegion, regionsInsideHomeRegion, lineObstacleRegions, polygonObstacleRegions, clusters, getLocalReferenceFrame().getTransformToWorldFrame(), visibilityGraphsParameters);
      ClusterTools.createClusterForHomeRegion(clusters, getLocalReferenceFrame().getTransformToWorldFrame(), homeRegion, visibilityGraphsParameters.getExtrusionDistance());

      clusterMgr = new ClusterManager();
      for (Cluster cluster : clusters)
      {
         clusterMgr.addCluster(cluster);
      }

      if (debug)
      {
         System.out.println("Extruding obstacles...");
      }

      // TODO The use of Double.MAX_VALUE for the observer seems rather risky. I'm actually surprised that it works.
      clusterMgr.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), visibilityGraphsParameters.getExtrusionDistance());

      for (Cluster cluster : clusters)
      {
         PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal(), visibilityGraphsParameters.getClusterResolution());
      }

      VisibilityMap visibilityMap = new VisibilityMap();
      HashSet<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(null, null, clusters);
      visibilityMap.setConnections(connectionsForMap);

      ArrayList<Connection> connections = new ArrayList<>();

      Iterator it = visibilityMap.getConnections().iterator();

      while (it.hasNext())
      {
         Connection connection = (Connection) it.next();
         connections.add(connection);
      }

      ArrayList<Connection> filteredConnections1 = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connections, homeRegion);
      ArrayList<Connection> filteredConnections2 = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(filteredConnections1, clusters);

      HashSet<Connection> sets = new HashSet<>();

      for (Connection connection : filteredConnections2)
      {
         sets.add(connection);
      }

      visibilityMap.setConnections(sets);

      localVisibilityMap = visibilityMap;
   }

   public VisibilityMap getLocalVisibilityGraph()
   {
      return localVisibilityMap;
   }

   public int getRegionId()
   {
      return homeRegion.getRegionId();
   }

   public List<Cluster> getClusters()
   {
      return clusters;
   }
}