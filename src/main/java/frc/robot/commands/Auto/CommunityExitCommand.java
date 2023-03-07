// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CommunityExitCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrain;
  private final double m_seconds;
  private final double m_speed;

  private long m_startTime;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param drivetrain The drivetrain used by this command.
   */
  public CommunityExitCommand(
    DrivetrainSubsystem drivetrain, // The drivetrain subsystem
    double seconds, // Number of seconds to drive
    double speed // motor speed %
  ) {
    m_drivetrain = drivetrain;
    m_seconds = seconds * 1000;
    m_speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Capture current time
    m_startTime = System.currentTimeMillis();
    // Make sure the drivetrain isn't moving
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move the drivetrain
    m_drivetrain.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Drive backwards for a number of seconds defined by the user
    return (System.currentTimeMillis() - m_startTime) >= m_seconds;
  }
}
