// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmWristSubsystem extends SubsystemBase {
    
  /* Arm Motor */
  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.NEO_ARM, MotorType.kBrushless);
  private final SparkMaxPIDController m_armMotorPidController;
  private final DutyCycleEncoder m_armMotorAbsEncoder;
  
  /* Wrist Motor */
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.NEO_WRIST, MotorType.kBrushless);
  private final SparkMaxPIDController m_wristMotorPidController;
  private final DutyCycleEncoder m_wristMotorAbsEncoder;

  NetworkTableEntry m_armPositionEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Position");
  NetworkTableEntry m_armMotorVoltageEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Voltage");
  NetworkTableEntry m_armMotorCurrentEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Arm Current");
  NetworkTableEntry m_wristPositionEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Position");
  NetworkTableEntry m_wristMotorVoltageEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Voltage");
  NetworkTableEntry m_wristMotorCurrentEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Wrist Current");

  /** Creates a new ArmWristSubsystem. */
  public ArmWristSubsystem() {

    /* Arm Motor Configuration */

    // Factory default configurations for all arm motors //
    m_armMotor.restoreFactoryDefaults();

    // Disable arm motors //
    m_armMotor.set(0);

    // Set neutral mode to brake on arm motor //
    m_armMotor.setIdleMode(IdleMode.kBrake);

    // Arm Motor Encoder //
    m_armMotorAbsEncoder = new DutyCycleEncoder(Constants.ARM_MOTOR_CHANNEL);
    m_armMotorAbsEncoder.reset();

    // Initialize PID controller and Encoder on arm motor //
    m_armMotorPidController = m_armMotor.getPIDController();

    // Set Arm PID Coefficients //
    m_armMotorPidController.setP(Constants.ArmSmartMotionConstants.kP);
    m_armMotorPidController.setI(Constants.ArmSmartMotionConstants.kI);
    m_armMotorPidController.setD(Constants.ArmSmartMotionConstants.kD);
    m_armMotorPidController.setIZone(Constants.ArmSmartMotionConstants.kIz);
    m_armMotorPidController.setFF(Constants.ArmSmartMotionConstants.kFF);
    m_armMotorPidController.setOutputRange(
        Constants.ArmSmartMotionConstants.kMinOutput, 
        Constants.ArmSmartMotionConstants.kMaxOutput
    );

    // Set Arm PID Smart Motion Coefficients //
    m_armMotorPidController.setSmartMotionMaxVelocity(
        Constants.ArmSmartMotionConstants.maxVel, Constants.ArmSmartMotionConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionMinOutputVelocity(
        Constants.ArmSmartMotionConstants.minVel, Constants.ArmSmartMotionConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionMaxAccel(
        Constants.ArmSmartMotionConstants.maxAcc, Constants.ArmSmartMotionConstants.SMART_MOTION_SLOT);
    m_armMotorPidController.setSmartMotionAllowedClosedLoopError(
        Constants.ArmSmartMotionConstants.allowedErr, Constants.ArmSmartMotionConstants.SMART_MOTION_SLOT);

    /* Wrist Motor Configuration */

    // Factory default configuration for all wrist motors //
    m_wristMotor.restoreFactoryDefaults();

    // Disable wrist motors //
    m_wristMotor.set(0);

    // Set neutral mode to brake on wrist motor //
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    // Wrist Motor Encoder //
    m_wristMotorAbsEncoder = new DutyCycleEncoder(Constants.WRIST_MOTOR_CHANNEL);
    m_wristMotorAbsEncoder.reset();

    // Initialize PID controller and Encoder on arm motor //
    m_wristMotorPidController = m_wristMotor.getPIDController();

    // Set Arm PID Coefficients //
    m_wristMotorPidController.setP(Constants.WristSmartMotionConstants.kP);
    m_wristMotorPidController.setI(Constants.WristSmartMotionConstants.kI);
    m_wristMotorPidController.setD(Constants.WristSmartMotionConstants.kD);
    m_wristMotorPidController.setIZone(Constants.WristSmartMotionConstants.kIz);
    m_wristMotorPidController.setFF(Constants.WristSmartMotionConstants.kFF);
    m_wristMotorPidController.setOutputRange(
        Constants.WristSmartMotionConstants.kMinOutput,
        Constants.WristSmartMotionConstants.kMaxOutput);

    // Set Arm PID Smart Motion Coefficients //
    m_wristMotorPidController.setSmartMotionMaxVelocity(
        Constants.WristSmartMotionConstants.maxVel, Constants.WristSmartMotionConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionMinOutputVelocity(
        Constants.WristSmartMotionConstants.minVel, Constants.WristSmartMotionConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionMaxAccel(
        Constants.WristSmartMotionConstants.maxAcc, Constants.WristSmartMotionConstants.SMART_MOTION_SLOT);
    m_wristMotorPidController.setSmartMotionAllowedClosedLoopError(
        Constants.WristSmartMotionConstants.allowedErr, Constants.WristSmartMotionConstants.SMART_MOTION_SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Added code to record arm and wrist data //
    m_armPositionEntry.setNumber(m_armMotorAbsEncoder.getAbsolutePosition());     // Get Arm Position
    m_armPositionEntry.setNumber(m_armMotor.getBusVoltage());                     // Get Arm Voltage
    m_armPositionEntry.setNumber(m_armMotor.getOutputCurrent());                  // Get Arm Current
    m_wristPositionEntry.setNumber(m_wristMotorAbsEncoder.getAbsolutePosition()); // Get Wrist Position
    m_wristPositionEntry.setNumber(m_wristMotor.getBusVoltage());                 // Get Wrist Voltage
    m_wristPositionEntry.setNumber(m_wristMotor.getOutputCurrent());              // Get Wrist Current

  }

  /* Arm Methods */
  public void moveArm(int position){
    m_armMotorPidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void rotateArm(double rotationSpeed){
    m_armMotor.set(rotationSpeed);
  }

  /* Wrist Methods */
  public void moveWrist(int position){
    m_wristMotorPidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void rotateWrist(double rotationSpeed){
    m_wristMotor.set(rotationSpeed);
  }
}
