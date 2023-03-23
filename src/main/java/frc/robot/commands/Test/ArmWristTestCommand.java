// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmWristSubsystem;

public class ArmWristTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmWristSubsystem m_armWrist;
  private final Supplier<Double> m_armRotationUpSpeed;
  private final Supplier<Double> m_armRotationDownSpeed;
  private final Supplier<Double> m_wristRotationSpeed;

  /** Creates a new ArmWristTestCommand. */
  public ArmWristTestCommand(
    ArmWristSubsystem armWrist,
    Supplier<Double> armRotationUpSpeed,
    Supplier<Double> armRotationDownSpeed,
    Supplier<Double> wristRotationSpeed
  ) {
    m_armWrist = armWrist;
    m_armRotationUpSpeed = armRotationUpSpeed;
    m_armRotationDownSpeed = armRotationDownSpeed;
    m_wristRotationSpeed = wristRotationSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armWrist.rotateArm(
      (m_armRotationUpSpeed.get() + m_armRotationDownSpeed.get()) * 0.4);
    m_armWrist.rotateWrist(m_wristRotationSpeed.get() * 0.25);
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
