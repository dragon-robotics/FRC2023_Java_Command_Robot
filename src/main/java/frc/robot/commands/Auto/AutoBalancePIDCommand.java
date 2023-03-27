// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancePIDCommand extends PIDCommand {
 /**
   * Turns to robot to the specified angle.
   *
   * @param targetPitchDegrees The pitch we want to land at
   * @param drivetrain The drive subsystem to use
   */
  public AutoBalancePIDCommand(double targetPitchDegrees, DrivetrainSubsystem drivetrain) {
    super(
        new PIDController(
            Constants.AUTO_BALANCE_P,
            Constants.AUTO_BALANCE_I,
            Constants.AUTO_BALANCE_D
        ),
        // Close loop on heading
        drivetrain::getPitch,
        // Set reference to target
        targetPitchDegrees,
        // Pipe output to turn robot
        output -> drivetrain.tankDriveVoltsAuto(output, output),
        // Require the drive
        drivetrain);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-30, 30);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.AUTO_BALANCE_DEG_TOL, Constants.AUTO_BALANCE_DEG_PER_S_TOL);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
