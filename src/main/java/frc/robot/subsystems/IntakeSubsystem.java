// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  DoubleSolenoid m_doublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid m_doublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public void compressorOff() {
    pcmCompressor.disable();
  }

  public void compressorOn() {
    pcmCompressor.enableDigital();
  }

}
