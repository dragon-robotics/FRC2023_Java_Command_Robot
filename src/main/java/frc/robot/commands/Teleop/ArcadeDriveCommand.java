// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** The arcade drive command that uses the drivetrain subsystem. */
public class ArcadeDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrain;
  private final Supplier<Double> m_speed;
  private final Supplier<Double> m_rotation;
  private final Supplier<Double> m_throttle;
  private final Supplier<Boolean> m_reverse;
  private final Supplier<Boolean> m_powerLimit80;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param drivetrain The drivetrain used by this command.
   */
  public ArcadeDriveCommand(
    DrivetrainSubsystem drivetrain,
    Supplier<Double> speed,
    Supplier<Double> rotation,
    Supplier<Double> throttle,
    Supplier<Boolean> reverse,
    Supplier<Boolean> powerLimit80
  ) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_rotation = rotation;
    m_throttle = throttle;
    m_reverse = reverse;
    m_powerLimit80 = powerLimit80;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure drivetrain is in coast //
    m_drivetrain.setNeutralMode(NeutralMode.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = m_throttle.get() > 0.4 ? 0.4 : 1 - m_throttle.get();
    double speed = m_reverse.get() ? -m_speed.get() * throttle : m_speed.get() * throttle;
    double rotation = m_rotation.get() * throttle;

    if (m_powerLimit80.get()){
      if (speed > 0.8) {
        speed = 0.8;
      }
    }
    NeutralMode brake = m_powerLimit80.get() ? NeutralMode.Brake : NeutralMode.Coast;
    m_drivetrain.setNeutralMode(brake);

    m_drivetrain.arcadeDrive(speed, rotation);
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
