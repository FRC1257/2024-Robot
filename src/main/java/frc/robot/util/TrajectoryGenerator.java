package frc.robot.util;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TrajectoryGenerator {

    public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        new PIDConstants(0, 0, 0, 0), 
        new PIDConstants(0, 0, 0, 0), 0, 0, null, 0)

    public static Command followTrajectory(PathPlannerPath path, Drive drive) {
        
    }
    
}