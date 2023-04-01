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
  private final Supplier<Double> m_armRotationSpeed;
  private final Supplier<Boolean> m_armReverse;
  private final Supplier<Double> m_wristRotationSpeed;

  /** Creates a new ArmWristTestCommand. */
  public ArmWristTestCommand(
    ArmWristSubsystem armWrist,
    Supplier<Double> armRotationSpeed,
    Supplier<Boolean> armReverse,
    Supplier<Double> wristRotationSpeed
  ) {
    m_armWrist = armWrist;
    m_armRotationSpeed = armRotationSpeed;
    m_armReverse = armReverse;
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
    double reverse = m_armReverse.get() ? -1.0 : 1.0;
    m_armWrist.rotateArm(m_armRotationSpeed.get() * 0.28 * reverse);
    m_armWrist.rotateWrist(m_wristRotationSpeed.get() * 0.15);
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
