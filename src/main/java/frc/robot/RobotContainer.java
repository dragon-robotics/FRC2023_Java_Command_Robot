// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoLoader.AutoCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ClawTestCommand;
import frc.robot.commands.CommunityExitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Joystick - 1st driver (driver) = channel 0, 2nd driver (operator) = channel 1
  private final Joystick m_driverController = new Joystick(Constants.DRIVER);

  // Create the auto loader class to load everything for us //
  private final AutoLoader m_autoLoader = new AutoLoader();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set default command to arcade drive when in teleop
    m_drivetrainSubsystem.setDefaultCommand(
        new ArcadeDriveCommand(
            m_drivetrainSubsystem,
            () -> -m_driverController.getRawAxis(Constants.STICK_LEFT_Y), // speed
            () -> m_driverController.getRawAxis(Constants.STICK_RIGHT_X), // turn
            () -> m_driverController.getRawAxis(Constants.TRIGGER_LEFT), // throttle
            () -> m_driverController.getRawButton(Constants.BUMPER_RIGHT) // reverse
        ));
    
    m_clawSubsystem.setDefaultCommand(
      new ClawTestCommand(
          m_clawSubsystem,
          () -> m_driverController.getRawButton(Constants.BTN_A), // extend
          () -> m_driverController.getRawButton(Constants.BTN_B) // retract
      ));
    
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_drivetrainSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_drivetrainSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_drivetrainSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoCommand command = m_autoLoader.getSelected();

    switch (command) {
      case NONE:
        return null;
      case COMMUNITY_EXIT:
        return new CommunityExitCommand(m_drivetrainSubsystem, 0.6, 2);
      default:
        return null;
    }
  }
}
