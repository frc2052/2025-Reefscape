package frc.robot.auto;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotContainer;
import frc.robot.util.io.Dashboard;

// note: simplifying to avoid HashMaps & use simple fields
// easier to debug & explain
    // we store the currently selected auto
    // if selection changes, we rebuild it

    // removed null map lookups, nested maps, etc.
    // also doesn't store auto from previous matches

public class AutoChooser extends SendableChooser<Auto>{

    // auto list
    private static final List<AutoProgram> AUTO_PROGRAMS = List.of(
        new AutoProgram(Auto.DRIVE_FORWARD, "Drive Forward", AutoFactory2::createDriveForwardAuto)
    );
    
    // factories --> need to swap geometries & starting pose
    private final AutoFactory2 blueFactory;
    private final AutoFactory2 redFactory;

    private Auto lastSelected = null;

    private Pair<Pose2d, Command> blueAuto = null;
    private Pair<Pose2d, Command> redAuto = null;

    private final Supplier<Double> waitSecondsEntrySupplier =
            () -> Dashboard.getInstance().getWaitSeconds();
    private final Supplier<Boolean> bumpNeededSupplier =
            () -> Dashboard.getInstance().getBumpNeeded();

    private double selectedWaitSeconds;
    private static double savedWaitSeconds;
    private static boolean savedBumpNeeded;

    private static LoggedNetworkBoolean waitSecondsSavedKey =
            new LoggedNetworkBoolean(DashboardConstants.WAIT_SECONDS_SAVED_KEY, false);

    private static LoggedNetworkString waitSecondsDisplay =
            new LoggedNetworkString(DashboardConstants.WAIT_SECONDS_DISPLAY_KEY, "DEFAULT - 0.0");

    public AutoChooser(RobotContainer robotcontainer){
        blueFactory = new AutoFactory2(DriverStation.Alliance.Blue, robotcontainer);
        redFactory = new AutoFactory2(DriverStation.Alliance.Red, robotcontainer); 
    
        // populate chooser
        for(AutoProgram program: AUTO_PROGRAMS){
            if(program.getAuto() == Auto.NO_AUTO){
                setDefaultOption(program.getName(), program.getAuto());
            } else {
                addOption(program.getName(), program.getAuto());
            }
        }

        Shuffleboard.getTab("Auto").add("Auto Chooser", this).withSize(3, 2);
    }

    public static AutoChooser create(final RobotContainer robotContainer) {

        var autoChooser = new AutoChooser(robotContainer);

        return autoChooser;
    }

    // TODO: call in disabledPeriodic()
    public void update() {
        Auto selected = getSelected();

        // update auto if chosen one changed
        if (selected != lastSelected
            || waitSecondsEntrySupplier.get() != savedWaitSeconds
            || bumpNeededSupplier.get() != savedBumpNeeded) {
            System.out.println("Rebuilding auto: " + selected);

            AutoProgram program = findProgram(selected);

            // build auto for both alliances
            // getCommand&Pose calls factory method & returns the Pair<Pose2d startPose, Command actualAuto>
            blueAuto = program.getCommandAndPose(blueFactory);
            redAuto = program.getCommandAndPose(redFactory);

            lastSelected = selected;
        }

        // update wait seconds
        if(waitSecondsEntrySupplier.get() != savedWaitSeconds){
            waitSecondsSavedKey.set(false);
            selectedWaitSeconds = waitSecondsEntrySupplier.get().doubleValue();
            savedWaitSeconds = selectedWaitSeconds;
            waitSecondsDisplay.set("Chosen Wait Seconds: " + savedWaitSeconds);
            waitSecondsSavedKey.set(true);
        }
        
        if(bumpNeededSupplier.get() != savedBumpNeeded){
            savedBumpNeeded = bumpNeededSupplier.get();
            System.out.println("BUMP NEEDE VALUE: " + savedBumpNeeded);
        }
    }

    public static boolean getBumpNeeded(){
        return savedBumpNeeded;
    }

    public static double getWaitSeconds(){
        return savedWaitSeconds;
    }

    // HELPERS

    // return auto command for current alliance
    // called @ autonomousInit
    // when you choose a new auto,
    // it creates a blueAuto and redAuto version,
    // WHEN AUTO BEGINS getAuto() is called and you run either red or blue @ runtime because both are updated
    public Command getAuto(){
        if(DriverStation.getAlliance().isPresent()){
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if(alliance == DriverStation.Alliance.Blue){
                return blueAuto.getSecond(); // returns command
            } else {
                return redAuto.getSecond();
            }
        }
        return null; // no auto yet
    }

    // starting pose for current auto
    // TODO: use to reset odom @ auto init
    public Pose2d getAutoStartPose(){
        if(DriverStation.getAlliance().isPresent()){
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if(alliance == DriverStation.Alliance.Blue){
                return blueAuto.getFirst(); // returns Pose2d
            } else {
                return redAuto.getFirst();
            }
        }
        return null; // no auto yet
    }

    // FIND THE AUTO PROGRAM that matches the enum - loop instead of streams from b4
    private AutoProgram findProgram(Auto dAuto){
        for (AutoProgram program: AUTO_PROGRAMS){
            if (program.getAuto() == dAuto){
                return program;
            } 
        }
        System.out.println("Could not find auto program for: " + dAuto.name());
        return null;
    }
}
