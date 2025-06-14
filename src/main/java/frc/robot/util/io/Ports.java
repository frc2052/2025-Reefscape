package frc.robot.util.io;

public class Ports {
    /*
     *  CAN IDS
     */
    /*
     * Bus: RoboRio
     */
    public static final int ARM_CANCODER_ID = 1;
    public static final int ARM_TALONFX_ID = 2;

    public static final int ARM_ROLLER_TALONFX_ID = 3;

    public static final int INTAKE_ROLLER_ID = 5;
    public static final int INTAKE_PIVOT_ID = 6;
    public static final int INTAKE_ENCODER_ID = 7;

    /*
     *  Bus: Krawlivore
     */
    public static final int ELEVATOR_FRONT_ID = 14;
    public static final int ELEVATOR_BACK_ID = 15;

    public static final int CLIMBER_ID = 16;

    /*
     *  DIO
     */
    public static final int CORAL_BEAM_BREAK_PIN = 4;
    public static final int INTAKE_BEAM_BREAK_ID = 3;

    public static final int LED_CHANNEL_1_PIN = 5;
    public static final int LED_CHANNEL_2_PIN = 6;
    public static final int LED_CHANNEL_3_PIN = 7;
    public static final int LED_CHANNEL_4_PIN = 8;
    public static final int LED_CHANNEL_5_PIN = 9;

    /*
     *  USB
     */

    public static final int GAMEPAD_PORT = 0;
    public static final int TRANSLATION_JOYSTICK_PORT = 0;
    public static final int ROTATION_JOYSTICK_PORT = 1;
    public static final int CONTROL_PANEL_PORT = 2;
}
