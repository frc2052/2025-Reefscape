package frc.robot.util.io;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.common.AutoFactory.Auto;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Dashboard {
    private final LoggedDashboardChooser<DriveMode> driveModeChooser = new LoggedDashboardChooser<>("Drive Mode");

    private final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<Auto>("Auto Mode");

    private final LoggedDashboardChooser<Double> waitSecondsChooser =
            new LoggedDashboardChooser<Double>("Wait Seconds");
    private final LoggedDashboardChooser<Boolean> smartDriveChooser =
            new LoggedDashboardChooser<Boolean>("Smart drive wanted");
    private final LoggedDashboardChooser<TargetAction> smartDriveActionChooser =
            new LoggedDashboardChooser<TargetAction>("Smart drive action");

    private final LoggedDashboardChooser<Boolean> bump = new LoggedDashboardChooser<Boolean>("Bump Needed");
    private final LoggedDashboardChooser<Boolean> lollipop = new LoggedDashboardChooser<Boolean>("Lollipop Order");

    private final LoggedDashboardChooser<Boolean> coastChooser = new LoggedDashboardChooser<Boolean>("Coast Out");
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable debugTable = networkTables.getTable("debug network tables tab");

    private final DoubleTopic waitTimeTopic = debugTable.getDoubleTopic("waitTime");
    private final DoubleSubscriber waitTimeSubscriber = waitTimeTopic.subscribe(0.0);

    private final DoubleTopic nudgeElevator = debugTable.getDoubleTopic("elevator nudge value");
    private final DoubleSubscriber nudgeElevatorSub = nudgeElevator.subscribe(0.0);

    private static Dashboard INSTANCE;

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    private Dashboard() {
        driveModeChooser.addDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);

        waitTimeTopic.publish().accept(0.0);
        nudgeElevator.publish().accept(0.0);

        autoChooser.addDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);

        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        waitSecondsChooser.addDefaultOption("None Chosen", 0.0);
        waitSecondsChooser.addOption("1 Second", 1.0);

        bump.addDefaultOption("No Bump Needed", false);
        bump.addOption("Bump Needed", true);

        lollipop.addDefaultOption("LEFT LOLLIPOP FIRST", true);
        lollipop.addOption("RIGHT LOLLIPOP FIRST", false);

        coastChooser.addDefaultOption("BRAKE", false);
        coastChooser.addOption("COAST", true);

        smartDriveChooser.addDefaultOption("yes", true);
        smartDriveChooser.addOption("no", false);

        smartDriveActionChooser.addDefaultOption("l3", TargetAction.L3);
        smartDriveActionChooser.addOption("l4", TargetAction.L4);
        smartDriveActionChooser.addOption("l2", TargetAction.L2);
        smartDriveActionChooser.addOption("l1", TargetAction.L1H);

        // choreo autos
        // choreoAutoChooser.addDefaultOption("NO AUTO", ChoreoAuto.NO_AUTO);
        // for (ChoreoAuto auto : ChoreoAuto.values()) {
        //     choreoAutoChooser.addOption(auto.name(), auto);
        // }
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
        } else if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Sendable) {
            Shuffleboard.getTab("main").add(key, (Sendable) value);
        }
    }

    public boolean isFieldCentric() {
        return driveModeChooser.get() == DriveMode.FIELD_CENTRIC;
    }

    public Auto getAuto() {
        return autoChooser.get();
    }

    public boolean getLeftLollipopFirst() {
        return lollipop.get();
    }

    public double getWaitSeconds() {
        return waitTimeSubscriber.get();
    }

    public double getElevatorNudgeValue() {
        return nudgeElevatorSub.get();
    }

    public boolean getBumpNeeded() {
        return bump.get();
    }

    public boolean getSmartDrivewanted() {
        return smartDriveChooser.get();
    }

    public TargetAction getSmartDriveAction() {
        return smartDriveActionChooser.get();
    }

    public boolean getCoastOut() {
        return coastChooser.get();
    }

    // Enums for Dashboard elements:
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }
}
