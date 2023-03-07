// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoLoader.AutoCommand;
import frc.robot.commands.Auto.CommunityExitCommand;
import frc.robot.commands.General.IntakeConeDownCommand;
import frc.robot.commands.General.IntakeConeUpCommand;
import frc.robot.commands.Teleop.ArcadeDriveCommand;
import frc.robot.commands.Test.ArmWristTestCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmWristSubsystem m_armWristSubsystem = new ArmWristSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Joystick - 1st driver (driver) = channel 0, 2nd driver (operator) = channel 1
  private final Joystick m_driverController = new Joystick(Constants.DRIVER);
  private final Joystick m_operatorController = new Joystick(Constants.OPERATOR);
  private final JoystickButton m_intakeConeUpButton = new JoystickButton(m_operatorController, Constants.BTN_A);
  private final JoystickButton m_intakeConeDownButton = new JoystickButton(m_operatorController, Constants.BTN_B);
  // private final Joystick m_operatorController = new Joystick(Constants.OPERATOR);

  // Create the auto loader class to load everything for us //
  private final AutoLoader m_autoLoader = new AutoLoader();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Initialize competetition shuffleboard
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
            () -> -m_driverController.getRawAxis(Constants.STICK_RIGHT_X), // turn
            () -> m_driverController.getRawAxis(Constants.TRIGGER_LEFT), // throttle
            () -> m_driverController.getRawButton(Constants.BUMPER_RIGHT) // reverse
        ));
    
    m_intakeConeDownButton.whileTrue(new IntakeConeDownCommand(m_intakeSubsystem));
    m_intakeConeUpButton.whileTrue(new IntakeConeUpCommand(m_intakeSubsystem));

    m_armWristSubsystem.setDefaultCommand(
      new ArmWristTestCommand(
        m_armWristSubsystem,
        () -> m_operatorController.getRawAxis(Constants.STICK_LEFT_Y),  // Arm Rotate
        () -> m_operatorController.getRawAxis(Constants.STICK_RIGHT_Y)  // Wrist Rotation
      )
    );
    
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
      case EXAMPLE_TRAJECTORY:
        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          10
        );

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
          Constants.kMaxSpeedMetersPerSecond,
          Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
            new Translation2d(1, 2),
            new Translation2d(3, 1),
            new Translation2d(2, 0),
            new Translation2d(3, -1),
            new Translation2d(1, -2)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(0, 0, new Rotation2d(Math.toRadians(-180))),
          // Pass config
          config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
          exampleTrajectory,
          m_drivetrainSubsystem::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          m_drivetrainSubsystem::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrainSubsystem::tankDriveVolts,
          m_drivetrainSubsystem
        );

        // Reset odometry to the starting pose of the trajectory.
        m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
      default:
        return null;
    }
  }
}
