// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmWristSubsystem extends SubsystemBase {
    
  /* Arm Motor */
  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.NEO_ARM, MotorType.kBrushless);
  private SparkMaxPIDController m_armMotorPidController;
  // private RelativeEncoder m_armMotorEncoder;
  
  /* Wrist Motor */
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.NEO_WRIST, MotorType.kBrushless);
  private SparkMaxPIDController m_wristMotorPidController;
  // private RelativeEncoder m_wristMotorEncoder;

  /** Creates a new ArmWristSubsystem. */
  public ArmWristSubsystem() {

    /* Arm Motor Configuration */

    // Factory default configurations for all arm motors //
    m_armMotor.restoreFactoryDefaults();

    // Disable arm motors //
    m_armMotor.set(0);

    // Set neutral mode to brake on arm motor //
    m_armMotor.setIdleMode(IdleMode.kBrake);

    // Set current limit on arm motor //
    // m_armMotor.setSmartCurrentLimit(30);

    // Set voltage compensation on arm motor //
    m_armMotor.enableVoltageCompensation(12);

    // Initialize PID controller and Encoder on arm motor //
    m_armMotorPidController = m_armMotor.getPIDController();
    // m_armMotorEncoder = m_armMotor.getEncoder();

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

    // Set current limit on wrist motor //
    // m_wristMotor.setSmartCurrentLimit(30);

    // Set voltage compensation on wrist motor //
    m_wristMotor.enableVoltageCompensation(12);

    // Initialize PID controller and Encoder on arm motor //
    m_wristMotorPidController = m_wristMotor.getPIDController();
    // m_wristMotorEncoder = m_wristMotor.getEncoder();

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
    // Get Wrist Position
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
