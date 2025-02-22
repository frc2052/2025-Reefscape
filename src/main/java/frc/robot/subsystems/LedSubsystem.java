package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoFactory.Auto;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.io.Dashboard;
import frc.robot.util.io.Ports;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins
 * and sent to the Arduino we used for controlling the patterns and colors
 */
public class LedSubsystem extends SubsystemBase {
  private static LedSubsystem INSTANCE;

  private final DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;
  private final RobotState state;

  private LEDStatusMode currentStatusMode;

  private boolean disableLEDs;
  private boolean robotDisabled;
  // this needs to be updated, and any references to these.
  private LedSubsystem() {
    // DIO outputs
    codeChannel1 = new DigitalOutput(Ports.LED_CHANNEL_1_PIN);
    codeChannel2 = new DigitalOutput(Ports.LED_CHANNEL_2_PIN);
    codeChannel3 = new DigitalOutput(Ports.LED_CHANNEL_3_PIN);
    codeChannel4 = new DigitalOutput(Ports.LED_CHANNEL_4_PIN);
    codeChannel5 = new DigitalOutput(Ports.LED_CHANNEL_5_PIN);
    robotDisabled = true;

    currentStatusMode = LEDStatusMode.OFF;

    state = RobotState.getInstance();

    // For manually inputing code to encode to DIO pins
    // SmartDashboard.putNumber("LED CODE", 0);
  }

  public static LedSubsystem
      getInstance() { // Method to allow calling this class and getting the single instance from
    // anywhere, creating the instance if the first time.
    if (INSTANCE == null) {
      INSTANCE = new LedSubsystem();
    }
    return INSTANCE;
  }
  // this needs to be updates
  public static enum LEDStatusMode {
    OFF(0),
    BLUE_DEFAULT(1),
    RED_DEFAULT(2),
    DISABLED(3),
    HAS_CORAL(4),
    REQUEST_LOADING(5),
    HAS_ALGAE(6),
    HOLD_BOTH(7),
    CLIMBING(8),
    DONE_CLIMBING(9),
    NO_AUTO(10),
    SPARKLE(11);

    private final int code;

    private LEDStatusMode(int code) {
      this.code = code;
    }

    public int getPositionTicks() {
      return code;
    }
  }

  @Override
  public void periodic() {
    int code = 0;
    if (!disableLEDs) {

      // disabled
      if (RobotState.getInstance().isRedAlliance()) {
        currentStatusMode = LEDStatusMode.RED_DEFAULT;
      } else if (!RobotState.getInstance().isRedAlliance()) {
        currentStatusMode = LEDStatusMode.BLUE_DEFAULT;
      } else {
        currentStatusMode = LEDStatusMode.SPARKLE;
      }

      if (DriverStation.isDisabled()) {
        // If disabled, gets the alliance color from the driver station and pulses that. Only pulses
        // color if connected to station or FMS, else pulses default disabled color (Firefl status
        // mode)
        Auto selected = Dashboard.getInstance().getAuto();
        if (selected == Auto.NO_AUTO || selected == null) {
          currentStatusMode = LEDStatusMode.NO_AUTO;
        } // Reaches here if DriverStation.getAlliance returns Invalid, which just
        // means it can't determine our alliance and we do cool default effect

        // autonomous LED status modes

      } else if (DriverStation.isAutonomous()) {
        // example implementation

        // if(RobotState.getInstance().getNoteHeldDetected()){
        //     currentStatusMode = LEDStatusMode.Has_Coral;
        // } else if (RobotState.getInstance().getIsShamperAtGoalAngle()){
        //     // currentStatusMode = LEDStatusMode.A;
        // } else {

        // }

      }

      // teleop LED status modes

      if (DriverStation.isTeleopEnabled()) {
        // example implementation
        // currentStatusMode = LEDStatusMode.HAS_ALGAE;
        if (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.HP) {
          currentStatusMode = LEDStatusMode.DONE_CLIMBING;
          // } else if (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L4)
          // {
          //   currentStatusMode = LEDStatusMode.DONE_CLIMBING;
          // } else if (Math.abs(ClimberSubsystem.getInstance().getSpeed()) > 0) {
          //   currentStatusMode = LEDStatusMode.CLIMBING;
        } else if (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L1H) {
          currentStatusMode = LEDStatusMode.SPARKLE;
        } else if (state.desiredReefFaceIsSeen()) {
          currentStatusMode = LEDStatusMode.REQUEST_LOADING;
          // } else if (state.getisAlignGoal()) {
          //   currentStatusMode = LEDStatusMode.HAS_ALGAE;
        }

        // shooting
        // if(RobotState.getInstance().getShooting()){
        //     if(!RobotState.getInstance().getNoteHeldDetected()){
        //         currentStatusMode = LEDStatusMode.DANGER;
        //                         } else if (RobotState.getInstance().getNoteHeldDetected() &&
        // RobotState.getInstance().getIsShamperAtGoalAngle() &&
        // RobotState.getInstance().getIsRotationOnTarget()){
        //         currentStatusMode = LEDStatusMode.SHOOTING_ON_TARGET;
        //     } else {
        //         currentStatusMode = LEDStatusMode.SHOOTING;
        //     }
        // }
        // //  aimed
        // else if (RobotState.getInstance().getNoteHeldDetected() &&
        // RobotState.getInstance().getIsShamperAtGoalAngle() &&
        // RobotState.getInstance().getIsRotationOnTarget()){
        //     currentStatusMode = LEDStatusMode.AIMING_ON_TARGET;
        // }
        // // aiming
        // else if (RobotState.getInstance().getIsVerticalAiming() ||
        // RobotState.getInstance().getIsHorizontalAiming())
        // {
        //     currentStatusMode = LEDStatusMode.AIMING;
        // } else if (RobotState.getInstance().getAmpIdle()){
        //     currentStatusMode = LEDStatusMode.AMP_IDLE;
        // }
        // else {
        //     if(RobotState.getInstance().getNoteHeldDetected()){
        //         currentStatusMode = LEDStatusMode.Has_Coral;
        //     } else if (RobotState.getInstance().getIsIntaking()){
        //         currentStatusMode = LEDStatusMode.INTAKE;
        //     } else {
        //         currentStatusMode = LEDStatusMode.OFF;
        //     }
        // }

        // }
      }

      code = currentStatusMode.code;
    } else {
      // LEDs are disabled
      code = 3;
    }

    // Code for encoding the code to binary on the digitalOutput pins
    Dashboard.getInstance().putData("Sending LED Code", code);
    codeChannel1.set((code & 1) > 0); // 2^0
    codeChannel2.set((code & 2) > 0); // 2^1
    codeChannel3.set((code & 4) > 0); // 2^2
    codeChannel4.set((code & 8) > 0); // 2^3
    codeChannel5.set((code & 16) > 0); // 2^4
  }

  public void setLEDStatusMode(LEDStatusMode statusMode) {
    if (!disableLEDs) {
      currentStatusMode = statusMode;
    }
  }

  public LEDStatusMode getLEDStatusMode() {
    return currentStatusMode;
  }

  public void clearStatusMode() {
    currentStatusMode = LEDStatusMode.OFF;
  }

  // Disables LEDs (turns them off)
  public void disableLEDs() {
    disableLEDs = true;
  }

  // Enables LEDs (turns them on)
  public void enableLEDs() {
    disableLEDs = false;
  }

  public boolean getRobotDisabled() {
    return robotDisabled;
  }
}
