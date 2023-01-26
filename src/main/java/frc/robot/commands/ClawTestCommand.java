// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawSubsystem m_claw;
  private final Supplier<Boolean> m_extend;
  private final Supplier<Boolean> m_retract;
  
  /** Creates a new ClawTestCommand. */
  public ClawTestCommand(
    ClawSubsystem claw,
    Supplier<Boolean> extend,
    Supplier<Boolean> retract
  ) {
    m_claw = claw;
    m_extend = extend;
    m_retract = retract;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.pneumaticsNeutral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_extend.get()) {
      m_claw.pneumaticsExtend();
    } else if (m_retract.get()) {
      m_claw.pneumaticsRetract();
    } else {
      m_claw.pneumaticsNeutral();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
