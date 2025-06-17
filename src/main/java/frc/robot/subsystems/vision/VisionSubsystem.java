package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;

    private static VisionSubsystem INSTANCE;

    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }

        return INSTANCE;
    }

    private VisionSubsystem() {

        io = new VisionIOLimelight();
        // if (RobotBase.isSimulation()) {
        //     io = new VisionIOSimPhoton();
        // } else {
        //     io = new VisionIOLimelight();
        // }
    }

    @Override
    public void periodic() {
        io.update();
    }
}
