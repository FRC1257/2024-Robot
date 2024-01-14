package frc.robot.util;

public class ArcadeDrive {

    // utility class
    private ArcadeDrive() {

    }

    // Code adapted from WPILib's DifferentialDrive
    public static double[] arcadeDrive(double speedForward, double speedTurn) {
        double forward = Math.copySign(speedForward * speedForward, speedForward);
        double turn = Math.copySign(speedTurn * speedTurn, speedTurn);

        if (Math.abs(forward) < 0.02) forward = 0.0;
        if (Math.abs(turn) < 0.02) turn = 0.0;

        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(turn)), forward);
        
        double speedLeft;
        double speedRight;

        if (forward >= 0.0) {
            if (turn >= 0.0) {
                speedLeft = maxInput;
                speedRight = forward - turn;
            }
            else {
                speedLeft = forward + turn;
                speedRight = maxInput;
            }
        } 
        else {
            if (turn >= 0.0) {
                speedLeft = forward + turn;
                speedRight = maxInput;
            }
            else {
                speedLeft = maxInput;
                speedRight = forward - turn;
            }
        }

        return new double[] {speedLeft, speedRight};
    }
}
