package frc.robot.util;
import static frc.robot.Constants.LookupTable;

import edu.wpi.first.math.geometry.Rotation2d;

public class Lookup {
    //Remaps a value from the range l1 to r1 to a range l2 to r2
    //e.g. map(5,0,10,10,20) returns 15
    private static double map(double value, double l1, double r1, double l2, double r2) {
        if (r1 - l1 == 0) {
           return l1; //if these bounds are the same number, return that number (prevents divide by 0)
       }
       return (value-l1) * (r2 - l2) / (r1 - l1) + l2;
    }

    //returns the remapped value based on the which column in the lookup table (e.g. func(distance, 1), would return remapped velocity)
    private static double getValueFromColumn(double distance, int column) {
      LookupTuner.updateMatrix(); //update the lookup table
      int lowerIndex = 0; //the index of the value in the lookup table that is just greater than distance
      
      //if the distance is below the lowest table distance, return a mapped value based on the least two table distances
      if (distance <= LookupTable[0][0]) {
        return map(distance, LookupTable[0][0], LookupTable[1][0], LookupTable[0][column], LookupTable[1][column]); //return mapped value based on the least two distances
      
      //if the distance is above the max table distance, return a mapped value based on the greatest two table distances
      } else if (distance > LookupTable[LookupTable.length-1][0]) {
        return map(distance, LookupTable[LookupTable.length-2][0], LookupTable[LookupTable.length-1][0], LookupTable[LookupTable.length-2][column], LookupTable[LookupTable.length-1][column]);
      
      } else {
        while (distance > LookupTable[lowerIndex][0]) {
          lowerIndex ++;
          if (lowerIndex == LookupTable.length) {
            lowerIndex -= 2;
            break;
          }
        }
        
        if (lowerIndex == LookupTable.length-1) {
          lowerIndex = LookupTable.length-2;
        }
        return map(distance, LookupTable[lowerIndex][0], LookupTable[lowerIndex+1][0], LookupTable[lowerIndex][column], LookupTable[lowerIndex+1][column]);
      }
    }
    // returns appropriate RPM based on distance
    public static double getRPM(double distance) {
      return getValueFromColumn(distance, 1); //1 is the index of RPM
    }
    //returns the proper angle based on distance
    public static double getAngle(double distance) {
      return Rotation2d.fromDegrees(getValueFromColumn(distance, 2)).getRadians(); //1 is the index of angle
    }
  }
