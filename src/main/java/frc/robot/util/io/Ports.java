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

  public static final int HAND_TALONFX_ID = 3;

  public static final int ALGAE_PIVOT_ID = 4;
  public static final int ALGAE_SCORING_ID = 5;
  public static final int ALGAE_ENCODER_ID = 6;

  /*
   *  Bus: Krawlivore
   */
  public static final int ELEVATOR_FRONT_ID = 14;
  public static final int ELEVATOR_BACK_ID = 15;

  public static final int CLIMBER_ID = 16;

  /*
   *  DIO
   */

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
