package frc.robot.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

// represents a single auto routine
// bridge between
// Auto.java enum identifier
// custom readable label for dashboard
// method reference to the auto in AutoFactory

// most important part
// FUNCTION<AutoFactory, Pair<Pose2d, Command>
// method reference!! --> stores a reference to a method w/o calling it
// when we need the command we call autofactory.apply(autoFactory)
// this is because there are TWO AutoFactory's for Red vs Blue alliance
// needed to grab startPose @ the beginning of an auto.

public class AutoProgram {
    private final Auto auto;
    private final String name; // for dashboard
    private final Function<AutoFactory2, Pair<Pose2d, Command>> command;

    public AutoProgram(Auto auto, String name, Function<AutoFactory2, Pair<Pose2d, Command>> command) {
        this.auto = auto;
        this.name = name;
        this.command = command;
    }

    public Auto getAuto() {
        return auto;
    }

    public String getName() {
        return name;
    }

    public Command getCommand(AutoFactory2 autoFactory) {
        return command.apply(autoFactory).getSecond();
    }

    public Pose2d getStartPose(AutoFactory2 autoFactory2) {
        return command.apply(autoFactory2).getFirst();
    }

    public Pair<Pose2d, Command> getCommandAndPose(AutoFactory2 autoFactory) {
        return command.apply(autoFactory);
    }
}
