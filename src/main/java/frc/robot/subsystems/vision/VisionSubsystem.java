package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final RobotState robotState = RobotState.getInstance();

    private static VisionSubsystem INSTANCE;

    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }

        return INSTANCE;
    }

    private VisionSubsystem() {
        if (RobotBase.isSimulation()) {
            io = new VisionIOSimPhoton();
        } else {
            io = new VisionIOLimelight();
        }
    }

    public boolean hasReefTarget() {
        return io.hasReefTarget();
    }
}
