// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANSparkMax m_intakeLead = new CANSparkMax(Constants.NEO_550_INTAKE_LEAD, MotorType.kBrushless);
  private final CANSparkMax m_intakeFollow = new CANSparkMax(Constants.NEO_550_INTAKE_FOLLOW, MotorType.kBrushless);

  /** Creates a new ClawSubsystem. */
  public IntakeSubsystem() {
    // Factory default configurations for all arm motors //
    m_intakeLead.restoreFactoryDefaults();
    m_intakeFollow.restoreFactoryDefaults();
    
    // Set default intake speed to be 0 //
    m_intakeLead.set(0);
    m_intakeFollow.set(0);

    // Make the follow motor follow the lead //
    m_intakeFollow.follow(m_intakeLead, true);
  }

  public void IntakeConeUp() {
    m_intakeLead.set(0.4);
    // m_intakeFollow.set(0.4);
  }

  public void IntakeConeDown() {
    m_intakeLead.set(-1);
    // m_intakeFollow.set(-1);
  }

  public void IntakeCubeUp() {
    m_intakeLead.set(-0.4);
    // m_intakeFollow.set(-0.4);
  }

  public void IntakeOff() {
    m_intakeLead.set(0);
    // m_intakeFollow.set(0);
  }
}
