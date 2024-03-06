package frc.robot.util.misc;

import static frc.robot.Constants.LookupTable;

public class LookupTuner {
    private static LoggedTunableNumber[][] lookupTable = new LoggedTunableNumber[LookupTable.length][LookupTable[0].length];

    public static void setupTuner() {
        for (int i = 0; i < LookupTable.length; i++) {
            for (int j = 1; j < LookupTable[i].length; j++) {
                System.out.print(LookupTable[i][j] + " ");
                String text = j == 1 ? "RPM" : "Angle";
                lookupTable[i][j] = new LoggedTunableNumber("LookupTable/" + "Meters-" + LookupTable[i][0] + "/" + text, LookupTable[i][j]);
            }
        }
    }

    public static double getMatrixValue(int i, int j) {
        return lookupTable[i][j].get();
    }

    public static void updateMatrix() {
        for (int i = 0; i < LookupTable.length; i++) {
            for (int j = 1; j < LookupTable[i].length; j++) {
                if (lookupTable[i][j].get() != LookupTable[i][j]) {
                    LookupTable[i][j] = lookupTable[i][j].get();
                }
            }
        }
    }
}
