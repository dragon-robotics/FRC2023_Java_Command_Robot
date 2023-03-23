// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ConeOutakeCommand extends CommandBase {
  /** Creates a new ConeOutake. */
private final IntakeSubsystem m_intake;
private final double m_speed;
private final double m_seconds;

private double m_startTime;

  public ConeOutakeCommand(
    IntakeSubsystem intake,
    double speed,
    double seconds
  ) {
    m_intake = intake;
    m_speed = speed;
    m_seconds = seconds * 1000;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_intake.rotateIntake(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.rotateIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.rotateIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_seconds;
  }
}
