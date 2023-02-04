// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private final Supplier<Double> m_rotationSpeed;
  private final Supplier<Double> m_counterRotationSpeed;
  private final Supplier<Boolean> m_extend;
  private final Supplier<Boolean> m_retract;

  /** Creates a new ArmTestCommand. */
  public ArmTestCommand(
    ArmSubsystem arm,
    Supplier<Double> rotationSpeed,
    Supplier<Double> counterRotationSpeed,
    Supplier<Boolean> extend,
    Supplier<Boolean> retract
  ) {
    m_arm = arm;
    m_rotationSpeed = rotationSpeed;
    m_counterRotationSpeed = counterRotationSpeed;
    m_extend = extend;
    m_retract = retract;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.rotateArm(m_rotationSpeed.get() - m_counterRotationSpeed.get());
    if (m_extend.get()) {
      m_arm.pneumaticsExtend();
    } else if (m_retract.get()) {
      m_arm.pneumaticsRetract();
    } else {
      m_arm.pneumaticsNeutral();
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
