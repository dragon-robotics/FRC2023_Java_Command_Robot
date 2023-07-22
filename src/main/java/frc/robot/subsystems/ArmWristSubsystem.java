// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.radiansToRotations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ArmWristSubsystem extends SubsystemBase {
    
  /* Arm Motor */
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmMotorConstants.NEO_ARM, MotorType.kBrushless);
  private final SparkMaxPIDController m_armMotorPidController;
  private final SparkMaxAbsoluteEncoder m_armMotorAbsEncoder;
  
  /* Wrist Motor */
  private final CANSparkMax m_wristMotor = new CANSparkMax(WristMotorConstants.NEO_WRIST, MotorType.kBrushless);
  private final SparkMaxPIDController m_wristMotorPidController;
  private final SparkMaxAbsoluteEncoder m_wristMotorAbsEncoder;

  NetworkTableEntry m_armPositionEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Position");
  NetworkTableEntry m_armMotorVoltageEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Voltage");
  NetworkTableEntry m_armMotorCurrentEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Current");
  NetworkTableEntry m_wristPositionEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Position");
  NetworkTableEntry m_wristMotorVoltageEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Voltage");
  NetworkTableEntry m_wristMotorCurrentEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Current");

  private Double targetArmPosition = null;
  private Double targetWristPosition = null;

  /** Creates a new ArmWristSubsystem. */
  public ArmWristSubsystem() {

    /* Arm Motor Configuration */

    // Factory default configurations for all arm motors //
    m_armMotor.restoreFactoryDefaults();

    // Disable arm motors //
    m_armMotor.set(0);

    // Set neutral mode to brake on arm motor //
    m_armMotor.setIdleMode(IdleMode.kBrake);

    // Voltage compensation and current limits //
    m_armMotor.enableVoltageCompensation(ArmMotorConstants.VOLTAGE_COMP);
    m_armMotor.setSmartCurrentLimit(ArmMotorConstants.SMART_CURRENT_LIMIT);

    // Configure soft limits //
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmMotorConstants.LIMIT_TOP);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmMotorConstants.LIMIT_BOTTOM);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Arm Motor Encoder //
    m_armMotorAbsEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Initialize PID controller and Encoder on arm motor //
    m_armMotorPidController = m_armMotor.getPIDController();

    // Set feedback device //
    m_armMotorPidController.setFeedbackDevice(m_armMotorAbsEncoder);

    // Set Arm PID Coefficients //
    m_armMotorPidController.setP(ArmMotorConstants.kP);
    m_armMotorPidController.setI(ArmMotorConstants.kI);
    m_armMotorPidController.setD(ArmMotorConstants.kD);
    m_armMotorPidController.setIZone(ArmMotorConstants.kIz);
    m_armMotorPidController.setFF(ArmMotorConstants.kFF);
    m_armMotorPidController.setOutputRange(
        ArmMotorConstants.kMinOutput, 
        ArmMotorConstants.kMaxOutput
    );

    // Set Arm PID Smart Motion Coefficients //
    m_armMotorPidController.setSmartMotionMaxVelocity(
        ArmMotorConstants.maxVel, ArmMotorConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionMinOutputVelocity(
        ArmMotorConstants.minVel, ArmMotorConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionMaxAccel(
        ArmMotorConstants.maxAcc, ArmMotorConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionAllowedClosedLoopError(
        ArmMotorConstants.allowedErr, ArmMotorConstants.SMART_MOTION_SLOT);

    /* Wrist Motor Configuration */

    // Factory default configuration for all wrist motors //
    m_wristMotor.restoreFactoryDefaults();

    // Disable wrist motors //
    m_wristMotor.set(0);

    // Set neutral mode to brake on wrist motor //
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    // Voltage compensation and current limits //
    m_wristMotor.enableVoltageCompensation(WristMotorConstants.VOLTAGE_COMP);
    m_wristMotor.setSmartCurrentLimit(WristMotorConstants.SMART_CURRENT_LIMIT);

    // Configure soft limits //
    m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, WristMotorConstants.LIMIT_TOP);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WristMotorConstants.LIMIT_BOTTOM);
    m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Wrist Motor Encoder //
    m_wristMotorAbsEncoder = m_wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Initialize PID controller and encoder on wrist motor //
    m_wristMotorPidController = m_wristMotor.getPIDController();

    // Set feedback device //
    m_wristMotorPidController.setFeedbackDevice(m_wristMotorAbsEncoder);

    // Set Arm PID Coefficients //
    m_wristMotorPidController.setP(WristMotorConstants.kP);
    m_wristMotorPidController.setI(WristMotorConstants.kI);
    m_wristMotorPidController.setD(WristMotorConstants.kD);
    m_wristMotorPidController.setIZone(WristMotorConstants.kIz);
    m_wristMotorPidController.setFF(WristMotorConstants.kFF);
    m_wristMotorPidController.setOutputRange(
        WristMotorConstants.kMinOutput,
        WristMotorConstants.kMaxOutput);

    // Set Arm PID Smart Motion Coefficients //
    m_wristMotorPidController.setSmartMotionMaxVelocity(
        WristMotorConstants.maxVel, WristMotorConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionMinOutputVelocity(
        WristMotorConstants.minVel, WristMotorConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionMaxAccel(
        WristMotorConstants.maxAcc, WristMotorConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionAllowedClosedLoopError(
        WristMotorConstants.allowedErr, WristMotorConstants.SMART_MOTION_SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (targetArmPosition != null) {
      // Calculate feed forward based on angle to counteract gravity
      double cosineScalar = Math.cos(getArmPosition());
      double feedForward = ArmMotorConstants.GRAVITY_FF * cosineScalar;
      m_armMotorPidController.setReference(
          radiansToEncoderRotations(targetArmPosition, "ARM"),
          ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
    }

    if (targetWristPosition != null) {
      // Calculate feed forward based on angle to counteract gravity
      double cosineScalar = Math.cos(getWristPosition());
      double feedForward = WristMotorConstants.GRAVITY_FF * cosineScalar;
      m_armMotorPidController.setReference(
          radiansToEncoderRotations(targetWristPosition, "WRIST"),
          ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
    }

    // Added code to record arm and wrist data //
    m_armPositionEntry.setNumber(m_armMotorAbsEncoder.getPosition());     // Get Arm Position
    m_armPositionEntry.setNumber(m_armMotor.getBusVoltage());             // Get Arm Voltage
    m_armPositionEntry.setNumber(m_armMotor.getOutputCurrent());          // Get Arm Current
    m_wristPositionEntry.setNumber(m_wristMotorAbsEncoder.getPosition()); // Get Wrist Position
    m_wristPositionEntry.setNumber(m_wristMotor.getBusVoltage());         // Get Wrist Voltage
    m_wristPositionEntry.setNumber(m_wristMotor.getOutputCurrent());      // Get Wrist Current
  }

  /* Arm Methods */

  /**
   * Moves the arm to a position. Zero is horizontal, up is positive.
   * There is no motor safety, so calling this will continue to move
   * the arm to this position, and hold it until another method is called.
   * 
   * @param radians position in radians
   */
  public void moveArmPosition(double radians){
    // Sets the target position in radians //
    targetArmPosition = radians;
  }

  /**
   * Moves the arm using duty cycle. There is no motor safety, so
   * calling this will continue to move until another method is
   * called.
   * 
   * @param rotationSpeed duty cycle [-1, 1]
   */
  public void rotateArm(double rotationSpeed){
    m_armMotor.set(rotationSpeed);
  }

  /**
   * Gets the arm position Zero is horizontal, up is positive
   * 
   * @return position in radians
   */
  public double getArmPosition() {
    return Units.rotationsToRadians(
        m_armMotorAbsEncoder.getPosition() + ArmMotorConstants.ENCODER_OFFSET);
  }

  /* Wrist Methods */

  /**
   * Moves the wrist to a position. Zero is horizontal, up is positive.
   * There is no motor safety, so calling this will continue to move
   * the wrist to this position, and hold it until another method is called.
   * 
   * @param radians position in radians
   */
  public void moveWristPosition(double radians){
    targetWristPosition = radians;
  }

  
  /**
   * Moves the wrist using duty cycle. There is no motor safety,
   * so calling this will continue to move until another method is
   * called.
   * 
   * @param rotationSpeed duty cycle [-1, 1]
   */
  public void rotateWrist(double rotationSpeed){
    m_wristMotor.set(rotationSpeed);
  }

  /**
   * Gets the wrist position Zero is horizontal, up is positive
   * 
   * @return position in radians
   */
  public double getWristPosition() {
    return Units.rotationsToRadians(m_wristMotorAbsEncoder.getPosition() + WristMotorConstants.ENCODER_OFFSET);
  }

  /**
   * Convert from arm or wrist position in radians to encoder rotations
   * 
   * @param radians arm or wrist position in radians
   * @return equivalent encoder position, in rotations
   */
  static double radiansToEncoderRotations(double radians, String appendage) {
    if (appendage == "ARM"){
      return radiansToRotations(radians) - ArmMotorConstants.ENCODER_OFFSET;
    } else {
      return radiansToRotations(radians) - WristMotorConstants.ENCODER_OFFSET;
    }
  }
}