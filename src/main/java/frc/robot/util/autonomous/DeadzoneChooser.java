package frc.robot.util.autonomous;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.drive.AllianceFlipUtil;

public class DeadzoneChooser {
    private LoggedDashboardChooser<List<Pair<Translation2d, Translation2d>>> deadzone;

    private Pair<Translation2d, Translation2d> otherAllianceDeadzone = new Pair<>(
        new Translation2d(9, 8.160), 
        new Translation2d(9, 0)
    );

    private final List<Pair<Translation2d, Translation2d>> threadTheNeedle = List.of( // blue alliance
        new Pair<>(
            new Translation2d(2.686, 8.171), 
            new Translation2d(4.585, 6.679)
        ),
        new Pair<>(
            new Translation2d(3.034, 5.563), 
            new Translation2d(4.585, 0))
    );

    // Tune this
    private final List<Pair<Translation2d, Translation2d>> blockTop = List.of( // blue alliance
        new Pair<>(
            new Translation2d(2.686, 8.171), 
            new Translation2d(4.585, 4.732)
        )
    );

    private final List<Pair<Translation2d, Translation2d>> blockBottom = List.of( // blue alliance
        new Pair<>(
            new Translation2d(3.625, 3.784), 
            new Translation2d(3.921, 0))
    );

    private final List<Pair<Translation2d, Translation2d>> blockStage = List.of( // blue alliance
        new Pair<> (
            new Translation2d(3.685, 5.615), 
            new Translation2d(5.731, 2.817)
        )
    );

    public DeadzoneChooser(String name){
        deadzone = new LoggedDashboardChooser<List<Pair<Translation2d, Translation2d>>>(name);
    }

    public void init() {
        deadzone.addOption("Thread the Needle", getBothAlliances(threadTheNeedle));
        deadzone.addOption("Block Top", getBothAlliances(blockTop));
        deadzone.addOption("Block Bottom", getBothAlliances(blockBottom));
        deadzone.addOption("Block Stage", getBothAlliances(blockStage));
        deadzone.addDefaultOption("None", getBothAlliances(List.of()));
    }

    public List<Pair<Translation2d, Translation2d>> getDeadzone(){
        return deadzone.get();
    }

    private List<Pair<Translation2d, Translation2d>> getBothAlliances(List<Pair<Translation2d, Translation2d>> blueObs){
        // AllianceFlipUtil.apply() is a method that flips the Translation2d objects in the pairs
        List<Pair<Translation2d, Translation2d>> obs = new ArrayList<>();

        for (Pair<Translation2d, Translation2d> pair : blueObs){
            obs.add(
                new Pair<> (
                    AllianceFlipUtil.apply(pair.getFirst()), 
                    AllianceFlipUtil.apply(pair.getSecond())
                )
            );
        }

        obs.add(
            new Pair<> (
                AllianceFlipUtil.apply(otherAllianceDeadzone.getFirst()), 
                AllianceFlipUtil.apply(otherAllianceDeadzone.getSecond())
            )
        );

        return obs;
    }
}
