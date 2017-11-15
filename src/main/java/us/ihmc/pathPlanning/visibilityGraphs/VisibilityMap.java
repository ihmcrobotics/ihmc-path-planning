package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.HashSet;

import us.ihmc.euclid.tuple3D.Point3D;

public class VisibilityMap
{
   private HashSet<Connection> connections;
   private HashSet<Point3D> vertices;

   public VisibilityMap()
   {
      connections = new HashSet<>();
   }

   public VisibilityMap(HashSet<Connection> connections)
   {
      this.connections = connections;
   }
   
   public void setConnections(HashSet<Connection> connections)
   {
      this.connections = connections;
   }
   
   public void addConnection(Connection connection)
   {
      connections.add(connection);
   }
   
   public void computeVertices()
   {
      vertices = new HashSet<>();
      for (Connection connection : connections)
      {
         vertices.add(connection.getPoint1());
         vertices.add(connection.getPoint2());
      }
   }
   
   public HashSet<Point3D> getVertices()
   {
      return vertices;
   }
   
   public HashSet<Connection> getConnections()
   {
      return connections;
   }
}
