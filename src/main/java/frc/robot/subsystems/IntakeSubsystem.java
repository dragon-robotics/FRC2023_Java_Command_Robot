// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax m_intakeL = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax m_intakeR = new CANSparkMax(8, MotorType.kBrushless);

  /** Creates a new ClawSubsystem. */
  public IntakeSubsystem() {}

  public void IntakeConeUp() {
    m_intakeL.set(0.4);
    m_intakeR.set(0.4);
  }

  public void IntakeConeDown() {
    m_intakeL.set(-1);
    m_intakeR.set(-1);
  }

  public void IntakeCubeUp() {
    m_intakeL.set(-0.4);
    m_intakeR.set(-0.4);
  }

  public void IntakeOff() {
    m_intakeL.set(0);
    m_intakeR.set(0);
  }
}
