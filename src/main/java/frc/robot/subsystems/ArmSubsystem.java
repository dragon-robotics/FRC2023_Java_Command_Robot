// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  WPI_TalonFX m_talonArmMotor = new WPI_TalonFX(6);

  DoubleSolenoid m_doublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid m_doublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
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

  public void rotateArm(double rotationSpeed){
    m_talonArmMotor.set(ControlMode.PercentOutput, rotationSpeed);
  }

  // Intake pneumatics commands
  public void pneumaticsExtend() {
    m_doublePCM1.set(kForward);
    m_doublePCM2.set(kForward);
  }

  public void pneumaticsRetract() {
    m_doublePCM1.set(kReverse);
    m_doublePCM2.set(kReverse);
  }

  public void pneumaticsNeutral() {
    m_doublePCM1.set(kOff);
    m_doublePCM2.set(kOff);
  }
}
