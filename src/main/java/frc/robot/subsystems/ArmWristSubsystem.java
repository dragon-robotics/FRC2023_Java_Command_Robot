// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWristSubsystem extends SubsystemBase {
    
  /* Arm Motor */
  private final WPI_TalonFX m_talonArmMotor = new WPI_TalonFX(6);
  
  /* Wrist Motor */
  private final CANSparkMax m_wristMotor = new CANSparkMax(0, MotorType.kBrushless);

  /* Intake Motor */
  private final CANSparkMax m_intakeMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotorRight = new CANSparkMax(0, MotorType.kBrushless);

  /** Creates a new ArmWristSubsystem. */
  public ArmWristSubsystem() {

    /* Arm Configuration */

    // Factory default configurations for all motors //
    m_talonArmMotor.configFactoryDefault();

    // Disable all motors //
    m_talonArmMotor.set(ControlMode.PercentOutput, 0);

    // Set neutral mode to coast on all motors //
    m_talonArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /* Arm Methods */
  public void moveArm(int level){

  }

  public void rotateArm(double rotationSpeed){
    m_talonArmMotor.set(ControlMode.PercentOutput, rotationSpeed);
  }

  /* Wrist Methods */
  public void moveWrist(int position){

  }

  public void rotateWrist(double rotationSpeed){

  }

  /* Intake Methods */
  public void moveIntake(double rotationSpeed){

  }
}
