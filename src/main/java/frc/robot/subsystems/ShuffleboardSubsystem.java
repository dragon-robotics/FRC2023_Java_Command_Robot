// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new Shuffleboard. */
  private final PowerDistribution m_pdp = new PowerDistribution();

  public ShuffleboardSubsystem() {
  }

  public void update() {
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Battery Voltage", m_pdp.getVoltage());
  }
}
