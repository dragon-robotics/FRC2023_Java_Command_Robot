// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmWristSubsystem extends SubsystemBase {
    
  /* Arm Motor */
  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.NEO_ARM, MotorType.kBrushless);
  
  /* Wrist Motor */
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.NEO_WRIST, MotorType.kBrushless);

  /* Intake Motor */
  // private final CANSparkMax m_intakeMotorLeft = new CANSparkMax(Constants.NEO_550_INTAKE_LEFT, MotorType.kBrushless);
  // private final CANSparkMax m_intakeMotorRight = new CANSparkMax(Constants.NEO_550_INTAKE_RIGHT, MotorType.kBrushless);

  /** Creates a new ArmWristSubsystem. */
  public ArmWristSubsystem() {

    /* Arm Motor Configuration */

    // Factory default configurations for all arm motors //
    m_armMotor.restoreFactoryDefaults();

    // Disable wrist motors //
    m_armMotor.set(0);

    // Set neutral mode to brake on wrist motor //
    m_armMotor.setIdleMode(IdleMode.kBrake);

    /* Wrist Motor Configuration */

    // Factory default configuration for all wrist motors //
    m_wristMotor.restoreFactoryDefaults();

    // Disable wrist motors //
    m_wristMotor.set(0);

    // Set neutral mode to brake on wrist motor //
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    /* Intake Motor Configuration */
    
    // // Factory default configuration for all intake motors //
    // m_intakeMotorLeft.restoreFactoryDefaults();
    // m_intakeMotorRight.restoreFactoryDefaults();

    // // Set neutral mode to coast on intake motors //
    // m_intakeMotorLeft.setIdleMode(IdleMode.kCoast);
    // m_intakeMotorRight.setIdleMode(IdleMode.kCoast);

    // // Set follower //
    // m_intakeMotorRight.follow(m_intakeMotorLeft, true);

    // // Disable intake motors //
    // m_intakeMotorLeft.set(0);
    // m_intakeMotorRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /* Arm Methods */
  public void moveArm(int level){

  }

  public void rotateArm(double rotationSpeed){
    m_armMotor.set(rotationSpeed);
  }

  /* Wrist Methods */
  public void moveWrist(int position){

  }

  public void rotateWrist(double rotationSpeed){
    m_wristMotor.set(rotationSpeed);
  }

  // /* Intake Methods */
  // public void rotateIntake(double rotationSpeed){
  //   m_intakeMotorLeft.set(rotationSpeed);
  // }
}
