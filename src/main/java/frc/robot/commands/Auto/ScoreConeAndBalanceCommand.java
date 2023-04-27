// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeAndBalanceCommand extends SequentialCommandGroup {
  /** Creates a new ScoreConeAndBalanceCommand. */
  public ScoreConeAndBalanceCommand(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new 4());
    addCommands(
      new ConeOutakeCommand(intake, -0.5, 1.0),
      new CommunityExitCommand(drivetrain, -0.5, 4),
      new WaitCommand(0.5),
      new CommunityExitCommand(drivetrain, 0.43, 2.5),
      new AutoBalancePIDCommand(drivetrain, 0),
      new WaitCommand(4)
      // new AutoBalanceWithIMUCommand(drivetrain, -0.1, 5)
      // new AutoBalanceCommand(drivetrain, 0.2, 0.2)
    );
  }
}
