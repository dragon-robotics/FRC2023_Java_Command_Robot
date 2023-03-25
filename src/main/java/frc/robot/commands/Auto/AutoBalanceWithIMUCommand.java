// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceWithIMUCommand extends CommandBase {

  private final DrivetrainSubsystem m_drivetrain;
  private final double m_speed;
  private final double m_seconds;

  private double m_startTime;

  /** Creates a new AutoBalanceWithIMUCommand. */
  public AutoBalanceWithIMUCommand(
    DrivetrainSubsystem drivetrain,
    double speed,
    double seconds
  ) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_seconds = seconds * 1000;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_drivetrain.arcadeDrive(0, 0);
    m_drivetrain.setNeutralMode(NeutralMode.Brake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
    // m_drivetrain.setNeutralMode(NeutralMode.Coast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Drive for user-defined seconds //
    return ((System.currentTimeMillis() - m_startTime) >= m_seconds) || 
          (Math.abs(m_drivetrain.getPitch()) <= 1);
  }
}
