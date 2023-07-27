// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Joystick Controller IDs, DRIVER = 0, OPERATOR = 1
  public static final int DRIVER = 0;
  public static final int OPERATOR = 1;

  // Joystick Analog Axis/Stick //
  public static final int STICK_LEFT_X = 0;
  public static final int STICK_LEFT_Y = 1;
  public static final int TRIGGER_LEFT = 2;
  public static final int TRIGGER_RIGHT = 3;
  public static final int STICK_RIGHT_X = 4;
  public static final int STICK_RIGHT_Y = 5;

  // Joystick Buttons //
  public static final int BTN_A = 1;
  public static final int BTN_B = 2;
  public static final int BTN_X = 3;
  public static final int BTN_Y = 4;
  public static final int BUMPER_LEFT = 5;
  public static final int BUMPER_RIGHT = 6;
  public static final int BTN_BACK = 7;
  public static final int BTN_START = 8;
  public static final int BTN_STICK_LEFT = 9;
  public static final int BTN_STICK_RIGHT = 10;

  // Motor ID Constants //

  // Drivetrain //
  public static final int MOTOR_LEFT_TOP = 2;
  public static final int MOTOR_LEFT_BOTTOM = 3;
  public static final int MOTOR_RIGHT_TOP = 4;
  public static final int MOTOR_RIGHT_BOTTOM = 1;

  // Intake //
  public static final int NEO_550_INTAKE_LEAD = 9;
  public static final int NEO_550_INTAKE_FOLLOW = 4;

  // PIGEON 2 ID //
  public static final int PIGEON2_ID = 1;

  // PID Constants for Smart Motion //
  // Arm //
  public static final class ArmMotorConstants {
    // Arm ID //
    public static final int NEO_ARM = 11;

    // Smart Motion Slot //
    public static final int SMART_MOTION_SLOT = 0;

    // Voltage Compensation //
    public static final double VOLTAGE_COMP = 10;

    // Smart Current Limit //
    public static final int SMART_CURRENT_LIMIT = 20;

    // Soft Limit - @TODO to be tuned //
    public static final float LIMIT_BOTTOM = 0.5804f;
    public static final float LIMIT_TOP = 0.8995f;

    // Absolute Encoder Offset - @TODO to be tuned //
    public static final double ENCODER_OFFSET = -0.7219101d;

    // Gravity Feedforward - @TODO to be tuned //
    public static final double GRAVITY_FF = 0.01;

    // PID coefficients - @TODO to be tuned //
    public static final double kP = 5e-5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final int maxRPM = 5700;

    // Smart Motion Coefficients - @TODO to be tuned //
    public static final int minVel = 0;
    public static final int maxVel = 2000;
    public static final int maxAcc = 1500;
    public static final int allowedErr = 0;
  }
  
  // Wrist //
  public static final class WristMotorConstants {
    // Wrist ID //
    public static final int NEO_WRIST = 10;

    // Smart Motion Slot //
    public static final int SMART_MOTION_SLOT = 0;

    // Voltage Compensation //
    public static final double VOLTAGE_COMP = 10;

    // Smart Current Limit //
    public static final int SMART_CURRENT_LIMIT = 20;

    // Soft Limit - @TODO to be tuned //
    public static final float LIMIT_BOTTOM = 0.5804f;
    public static final float LIMIT_TOP = 0.8995f;

    // Absolute Encoder Offset - @TODO to be tuned //
    public static final double ENCODER_OFFSET = -0.7219101d;

    // Gravity Feedforward - @TODO to be tuned //
    public static final double GRAVITY_FF = 0.1;

    // PID coefficients - @TODO to be tuned
    public static final double kP = 0.0007;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final int maxRPM = 5700;

    // Smart Motion Coefficients - @TODO to be tuned
    public static final int minVel = 0;
    public static final int maxVel = 2000;
    public static final int maxAcc = 1500;
    public static final int allowedErr = 0;
  }

  // Auto Balance Constants //
  public static final double AUTO_BALANCE_P = 0.11;
  public static final double AUTO_BALANCE_D = 0.0;
  public static final double AUTO_BALANCE_I = 0.0;
  public static final double AUTO_BALANCE_DEG_TOL = 3.5;
  public static final double AUTO_BALANCE_DEG_PER_S_TOL = 1.5;

  // Robot Measurement Constants //
  public static final double TRACK_WIDTH_METERS = 0.546;
  public static final double WHEEL_DIAMETER_METERS = 0.1524;
  public static final double GEAR_RATIO = 10.71;
  public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
  public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

  // Encoder Constants //
  public static final int ENCODER_CPR = 2048;
  public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_CIRCUMFERENCE_METERS)
      / ((double) ENCODER_CPR * GEAR_RATIO);

  // Differential Drive Kinematics //
  public static final DifferentialDriveKinematics DIFF_DRIVE_KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);

  // Feedforward Feedback Gain //
  public static final double ksVolts = 0.53308;
  public static final double kvVoltSecondsPerMeter = 2.46;
  public static final double kaVoltSecondsSquaredPerMeter = 0.1619;
  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 1.9925;

  // DifferentialDriveKinematics //
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);

  // Max Trajectory Velocity/Acceleration //
  public static final double kMaxSpeedMetersPerSecond = 1;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1;

  // Ramsete Parameters //
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
}
