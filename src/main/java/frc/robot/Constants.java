// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


public final class Constants {
    
   // CAN bus IDs for the motor controllers for the drive train
   public static final int MOTOR_FRONT_LEFT = 10;
   public static final int MOTOR_FRONT_RIGHT = 12;
   public static final int MOTOR_REAR_LEFT = 11;
   public static final int MOTOR_REAR_RIGHT = 13;

   // CANID for Manip Controllers
   public static final int INTAKE_CANID = 22;
   public static final int FUNNEL_MOTOR_CANID = 23;
   public static final int LOWER_FUNNEL_SIDE_INDEX_CANID = 25;
   public static final int LOWER_FAR_SIDE_INDEX_CANID = 24;
   public static final int UPPER_BELT_CANID = 15;
   public static final int LOADING_BELT_CANID = 16;
   public static final int SHOOTER_CANID = 14;
   public static final int COLOR_WHEEL_MOTOR_CANID = 888;
   public static final int DS_REVERSE_CHANNEL = 1;
   public static final int DS_FORWARD_CHANNEL = 0;
   public static final int TURRET_CANID = 26;
   public static final int CLIMBER_CANID = 20;

   // USB IDs in the DS for the controller.
   public static final int LEFT_JOYSTICK = 0;
   public static final int RIGHT_JOYSTICK = 1;
   public static final int GAMEPAD_PORT = 2;

   // Button ID values on the Joystick
   public static final int JOYSTICK_TRIGGER = 1;
   public static final int JOYSTICK_BOTTOM_BUTTON = 2;
   public static final int JOYSTICK_CENTER_BUTTON = 3;
   public static final int JOYSTICK_LEFT_BUTTON = 4;
   public static final int JOYSTICK_RIGHT_BUTTON = 5;

   // Axes values for Joysticks
   public static final int JOYSTICK_X_AXIS = 0;
   public static final int JOYSTICK_Y_AXIS = 1;
   public static final int JOYSTICK_SLIDER_AXIS = 2;

   // Button ID values on the 3D Joystick
   public static final int JOYSTICK_3D_TRIGGER = 1;
   public static final int JOYSTICK_3D_THUMB_BUTTON = 2;
   public static final int JOYSTICK_3D_LOWER_LEFT_BUTTON = 3;
   public static final int JOYSTICK_3D_LOWER_RIGHT_BUTTON = 4;
   public static final int JOYSTICK_3D_UPPER_LEFT_BUTTON = 5;
   public static final int JOYSTICK_3D_UPPER_RIGHT_BUTTON = 6;

   // Axes values for Joysticks
   public static final int JOYSTICK_3D_X_AXIS = 0;
   public static final int JOYSTICK_3D_Y_AXIS = 1;
   public static final int JOYSTICK_3D_Z_AXIS = 2;
   public static final int JOYSTICK_3D_SLIDER_AXIS = 3;

   // Button values for gamepad
   public static final int A_BUTTON = 1;
   public static final int B_BUTTON = 2;
   public static final int X_BUTTON = 3;
   public static final int Y_BUTTON = 4;
   public static final int GP_L_BUTTON = 5;
   public static final int GP_R_BUTTON = 6;

   // Axes values for gamepad
   public static final int GP_L_X_AXIS = 0;
   public static final int GP_L_Y_AXIS = 1;

   public static final int GP_L_TRIGGER = 2;
   public static final int GP_R_TRIGGER = 3;

   public static final int GP_R_X_AXIS = 4;
   public static final int GP_R_Y_AXIS = 5;

   // IDs for pneumatic controls
   public static final int PCM = 5;
   public static final int SOLENOID = 1;

   public static final int RECEIVER_SOCKET = 5555;

   public static final double JOYSTICK_DEADZONE = 0.2;
   public static final double RAMP_RATE = 0.2;

    public static final double CLICKS_PER_INCH = 0.46683;
    public static final double WHEEL_BASE = CLICKS_PER_INCH * 21.5;

    public static final double COLOR_WHEEL_SPEED = 0.75;

    public static final double POSITION_TOLERENCE = 4;
    public static final double ANGLE_TOLERENCE = 4;

    public static final int BALL_POS_0_CHANNEL = 0;
    public static final int BALL_POS_1_CHANNEL = 1;
    public static final int BALL_POS_2_CHANNEL = 2;
    public static final int BALL_POS_3_CHANNEL = 3;
    public static final int BALL_POS_4_CHANNEL = 4;

    public static final int TURRET_ENCODER_CHANNEL = 9;

    public static final int SHOOTER_RPM_TOLERANCE = 100;

    public static final int JETSON_RINGLIGHT_CHANNEL = 7;
    



}

