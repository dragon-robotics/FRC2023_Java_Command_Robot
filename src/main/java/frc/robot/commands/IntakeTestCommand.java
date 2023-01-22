// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private final Supplier<Boolean> m_extend;
  private final Supplier<Boolean> m_retract;
  
  /** Creates a new IntakeTestCommand. */
  public IntakeTestCommand(
    IntakeSubsystem intake,
    Supplier<Boolean> extend,
    Supplier<Boolean> retract
  ) {
    m_intake = intake;
    m_extend = extend;
    m_retract = retract;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.pneumaticsNeutral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_extend.get()) {
      m_intake.pneumaticsExtend();
    } else if (m_retract.get()) {
      m_intake.pneumaticsRetract();
    } else {
      m_intake.pneumaticsNeutral();
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
