package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import javafx.application.Platform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FakeTest
{
   private static final File defaultFile = new File("./Data/20171026_PlanarRegion/20171026_133736_PlanarRegion");
   private static final File defaultFile1 = new File("./Data/20171026_PlanarRegion/");
   
   private Point3D start;
   private Point3D goal;

   @Test(timeout = 30000)
   public void testPassing() throws Exception
   {
      PlanarRegionsList planarRegionData = null;

      if (defaultFile != null)
         planarRegionData = PlanarRegionDataImporter.importPlanRegionData(defaultFile);

      if (planarRegionData == null)
         Platform.exit();

      readStartGoalParameters(defaultFile1.getAbsolutePath() + "/StartGoalParameters.txt");
      
      List<PlanarRegion> regions = planarRegionData.getPlanarRegionsAsList();
      ArrayList<PlanarRegion> filteredRegions = new ArrayList<>();

      for (PlanarRegion region : regions)
      {
         if (region.getConcaveHullSize() > 2)
         {
            filteredRegions.add(region);
         }
      }

      NavigableRegionsManager manager = new NavigableRegionsManager(filteredRegions, null);

      
      System.out.println(start + "     " + goal);
      
      Point3D start = new Point3D(-2.419, 0.312, 0.001);
      Point3D goal = new Point3D(0.792, 0.257, 0.001);

      ArrayList<Point3D> path = (ArrayList<Point3D>) manager.calculateBodyPath(start, goal);

      assertTrue(path.size() == 2);
   }

   public ArrayList<Point3D> readStartGoalParameters(String fileName)
   {
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         int index = 0;

         while ((sCurrentLine = br.readLine()) != null)
         {
            if(sCurrentLine.contains("Start=") && sCurrentLine.contains("Goal="))
            {
               String tempStart = sCurrentLine.substring(6, sCurrentLine.indexOf(",Goal="));
               start = getPoint3DFromStringSet(tempStart);
               
               String tempGoal = sCurrentLine.substring(sCurrentLine.indexOf(",Goal=") + 6);
               goal = getPoint3DFromStringSet(tempGoal);
            }
         }
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }
      }
      return null;

   }
   
   private Point3D getPoint3DFromStringSet(String set)
   {
      
      double x = Double.parseDouble(set.substring(0, set.indexOf(",")));
      set = set.substring(set.indexOf(",") + 1);
      double y = Double.parseDouble(set.substring(0, set.indexOf(",")));
      set = set.substring(set.indexOf(",") + 1);
      System.out.println(set);
      double z = Double.parseDouble(set.substring(0));
      
      return new Point3D(x,y,z);
   }
}
