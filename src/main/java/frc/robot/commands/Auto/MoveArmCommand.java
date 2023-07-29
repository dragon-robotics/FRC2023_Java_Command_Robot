// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmWristSubsystem;

public class MoveArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmWristSubsystem m_armWrist;
  private double m_speed;
  private double m_milliseconds;

  private long m_startTime;

  /** Creates a new MoveArmCommand. */
  public MoveArmCommand(
    ArmWristSubsystem armWrist,
    double speed,
    double seconds
  ) {
    m_armWrist = armWrist;
    m_speed = speed;
    m_milliseconds = seconds * 1000;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Capture current time
    m_startTime = System.currentTimeMillis();
    m_armWrist.stopArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armWrist.rotateArm(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armWrist.rotateArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Move arm for a number of seconds defined by the user
    return (System.currentTimeMillis() - m_startTime) >= m_milliseconds;
  }
}
