package frc.robot.auto;

// compile time constants for AutoPrograms
    // simple identifier for each auto
    // instead of the old system which stored class references
    // this is more lightweight & doesn't load classes at swap
// naming convention
    // starting position (LEFT, RIGHT)
    // scoring sequence
    // LOLI or BACKUP

// Adding new autos?
    // 1. add enum value here
    // 2. create an AutoProgram in AutoChooser.AUTO_PROGRAMS
    // 3. create the method & "actual auto" in AutoFactory

public enum Auto {
    NO_AUTO,

    DRIVE_FORWARD

    // BACKUP_MIDDLE_L1, //
    // DEAD_RECKONING,//

    // H4_ALGAE_GH_EF_IJ, //
    // MIDDLE_H4, //

    // LEFT_3_CORAL_JKL, //
    // RIGHT_3_CORAL_EDC, //

    // LOLI_LEFT_LEFT_FIRST, //
    // LOLI_RIGHT_RIGHT_FIRST, //
}
