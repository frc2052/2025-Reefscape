package frc.robot.auto;

import java.util.function.Function;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.common.AutoFactory;

// represents a single auto routine 
// bridge between 
    // Auto.java enum identifier
    // custom readable label for dashboard
    // method reference to the auto in AutoFactory

// most important part
    // FUNCTION<AutoFactory, Pair<Pose2d, Command>
    // method reference!! --> stores a reference to a method w/o calling it
    // when we need the command we call autofactory.apply(autoFactory)
    // then it applies the 2nd param (arguments) to first param (method)

public class AutoProgram {
    private final Auto auto; 
    private final String name; // for dashboard
    private final Function<AutoFactory, Pair<Pose2d, Command>> command; 

    public AutoProgram(
        Auto auto,
        String name,
        Function<AutoFactory, Pair<Pose2d, Command>> command
    ){
        this.auto = auto;
        this.name = name;
        this.command = command;
    }

    public Auto getAuto(){
        return auto;
    }

    public String getName(){
        return name;
    }

    // construct the command for this auto using provided AutoFactory
        // call command.apply(autofactory)
        // this executes the method reference  
            // method returns a Pair<Pose2d, Command>
        // .getSecond() grabs only the Command & returns it!!
    
    public Command getCommand(AutoFactory autoFactory){
        return command.apply(autoFactory).getSecond();
    }

    // get starting pose to reset odom
    // extracts Pose2d stored in the AutoFactory method

    public Pose2d getStartPose(AutoFactory autoFactory){
        return command.apply(autoFactory).getFirst();
    } 
}
