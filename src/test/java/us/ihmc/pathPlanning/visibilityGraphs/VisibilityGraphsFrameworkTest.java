package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import javafx.application.Platform;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class VisibilityGraphsFrameworkTest
{
   private File fileLocationForPlanarRegions = new File("./Data/20171026_133736_PlanarRegion/20171026_133736_PlanarRegion");
   private File fileLocationForStartGoalParameters = new File("./Data/20171026_133736_PlanarRegion/");

   private Point3D start;
   private Point3D goal;

   private boolean debug = false;

   @Before
   private void setup()
   {
      debug = debug && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test(timeout = 30000)
   @Ignore
   public void testASolutionExists() throws Exception
   {
      File fileLocationsForAllData = new File("./Data");

      if(debug)
         PrintTools.info("Unit test files found: " + fileLocationsForAllData.listFiles().length);

      File[] files = fileLocationsForAllData.listFiles();
      for (int i = 0; i < files.length; i++)
      {
         System.out.println("\n\nProcessing file: " + files[i].getName());
         String name = files[i].getName().replace("_UnitTest", "");
         fileLocationForStartGoalParameters = new File("./Data/" + files[i].getName() + "/");
         fileLocationForPlanarRegions = new File("./Data/" + files[i].getName() + "/" + name + "/");
         
         PlanarRegionsList planarRegionData = null;

         if (fileLocationForPlanarRegions != null)
            planarRegionData = PlanarRegionDataImporter.importPlanRegionData(fileLocationForPlanarRegions);

         if (planarRegionData == null)
            Platform.exit();
         
         File fileFolder = new File("./Data/" + files[i].getName() + "/" );
         File[] files1 = fileFolder.listFiles();
         
         for(int j = 0; j < files1.length; j++)
         {
            if(files1[j].getName().contains("UnitTestParameters"))
            {
               if(debug)
               {
                  PrintTools.info("Found test file: " + files1[j].getName());
                  PrintTools.info("Running test for : " + files1[j].getName());
               }

               readStartGoalParameters(fileLocationForStartGoalParameters.getAbsolutePath() + "/" + files1[j].getName());

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

               ArrayList<Point3D> path = (ArrayList<Point3D>) manager.calculateBodyPath(start, goal);
               
               assertTrue("Path is null!", path != null);
               assertTrue("Path does not contain any Wps", path.size() > 0);
               testPathSize(fileLocationForStartGoalParameters.getAbsolutePath() + "/" + files1[j].getName(), path);
            }
         }
      }
   }

   public void testPathSize(String fileName, ArrayList<Point3D> path)
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
            if (sCurrentLine.contains("<PathSize,") && sCurrentLine.contains(",PathSize>"))
            {
               double pathSize = Double.parseDouble(sCurrentLine.substring(10, sCurrentLine.indexOf(",PathSize>")));
               assertTrue("Path size is not equal",path.size() == pathSize);
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
            if (sCurrentLine.contains("<Start,") && sCurrentLine.contains("Start>"))
            {
               String tempStart = sCurrentLine.substring(7, sCurrentLine.indexOf(",Start>"));
               start = getPoint3DFromStringSet(tempStart);
            }
            
            if (sCurrentLine.contains("<Goal,") && sCurrentLine.contains(",Goal>"))
            {
               String tempGoal = sCurrentLine.substring(6, sCurrentLine.indexOf(",Goal>"));
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
      double z = Double.parseDouble(set.substring(0));

      return new Point3D(x, y, z);
   }
}
