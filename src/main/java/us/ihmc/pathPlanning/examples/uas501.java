package us.ihmc.pathPlanning.examples;

public class uas501
{
   public static void main(String args[])
   {
      double d = 6; // in ft
      double J = 0.6; //advance ratio
      double cp = 0.05; //Cp
      double vKias = 60; //in knots
      double densityAtAltitude = 1.028E-3; //from MIL-STD-3013 - Standard day atmosphere
      double densitySeaLevel = 0.002377;

      double densityRatio = densityAtAltitude / densitySeaLevel;
      double vInFtPerSec = vKias * 1.69; //in ft/s
      double vTrue = vInFtPerSec / Math.sqrt(densityRatio); //in ft/s
      double n = vTrue / (J * d);
      double pShaft = cp * densityAtAltitude * Math.pow(n, 3.0) * Math.pow(d, 5.0);
      double hp = pShaft * 60 * (1.0 / 33000.0); // in hp

      System.out.println("Shaft power absorbed by propller [hp]: " + hp);
      
      double speedOfSoundAtAlt = 1011.7; //from MIL-STD-3013 - Standard day atmosphere
      double machTip = Math.sqrt((Math.pow((Math.PI*d*n), 2.0) + Math.pow(vInFtPerSec, 2.0)))/speedOfSoundAtAlt;
      
      System.out.println("Mach Propeller Tip: " + machTip);
   }
}
