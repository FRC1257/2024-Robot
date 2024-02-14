import static frc.robot.Constants.LookupTable;


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
       int upperIndex = LookupTable.length-1; //the index of the value in the lookup table that is just greater than distance
       while (distance <= LookupTable[upperIndex][0]) {
           upperIndex --;
       }
       return map(distance, LookupTable[upperIndex-1][0], LookupTable[upperIndex][0], LookupTable[upperIndex-1][column], LookupTable[upperIndex][column]);
   }
   public static double getVelocity(double distance) {
       getValueFromColumn(distance, 1); //1 is the index of velocity
   }
   //returns the proper velocity based on
   public static double getAngle(double distance) {
       getValueFromColumn(distance, 2); //1 is the index of velocity
   }
}

