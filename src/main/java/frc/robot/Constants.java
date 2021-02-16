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
    
    // TELEOP
    // CAN bus IDs for the motor controllers for the drive train 
    public static final int MOTOR_FRONT_LEFT = 10;
    public static final int MOTOR_FRONT_RIGHT = 12;
    public static final int MOTOR_REAR_LEFT = 11;
    public static final int MOTOR_REAR_RIGHT = 13;
    // IDs for pneumatic controls
    public static final double RAMP_RATE = 0.2;

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


    // Button ID values on the Joystick
     public static final int JOYSTICK_TRIGGER = 1;
     public static final int JOYSTICK_BOTTOM_BUTTON = 2;
     public static final int JOYSTICK_CENTER_BUTTON = 3;
     public static final int JOYSTICK_LEFT_BUTTON = 4;
     public static final int JOYSTICK_RIGHT_BUTTON = 5;

     // USB IDs in the DS for the controller.
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int GAMEPAD_PORT = 2;


    // IDs for pneumatic controls
    public static final int PCM = 5;
    public static final int SOLENOID = 1;



    



}

