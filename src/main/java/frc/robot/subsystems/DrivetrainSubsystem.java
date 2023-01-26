// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

  // Declare subsystem attribute/components //

  // Motor Controllers //
  WPI_TalonFX m_talonLeftLead = new WPI_TalonFX(Constants.MOTOR_LEFT_TOP);
  WPI_TalonFX m_talonLeftFollow = new WPI_TalonFX(Constants.MOTOR_LEFT_BOTTOM);
  WPI_TalonFX m_talonRightLead = new WPI_TalonFX(Constants.MOTOR_RIGHT_TOP);
  WPI_TalonFX m_talonRightFollow = new WPI_TalonFX(Constants.MOTOR_RIGHT_BOTTOM);
  DifferentialDrive m_drive = new DifferentialDrive(m_talonLeftLead, m_talonRightLead);

  // Gyro //
  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // Factory default configurations for all motors //
    m_talonLeftLead.configFactoryDefault();
    m_talonLeftFollow.configFactoryDefault();
    m_talonRightLead.configFactoryDefault();
    m_talonRightFollow.configFactoryDefault();

    // Disable all motors //
    m_talonLeftLead.set(ControlMode.PercentOutput, 0);
    m_talonLeftFollow.set(ControlMode.PercentOutput, 0);
    m_talonRightLead.set(ControlMode.PercentOutput, 0);
    m_talonRightFollow.set(ControlMode.PercentOutput, 0);

    // Set neutral mode to brake on all motors //
    m_talonLeftLead.setNeutralMode(NeutralMode.Coast);
    m_talonLeftFollow.setNeutralMode(NeutralMode.Coast);
    m_talonRightLead.setNeutralMode(NeutralMode.Coast);
    m_talonRightFollow.setNeutralMode(NeutralMode.Coast);

    // Set our followers to follow the lead motor //
    m_talonLeftFollow.follow(m_talonLeftLead);
    m_talonRightFollow.follow(m_talonRightLead);

    // Set our follower's inverted to be opposite of the master //
    m_talonLeftFollow.setInverted(InvertType.FollowMaster);
    m_talonRightFollow.setInverted(InvertType.FollowMaster);

    // Set our lead motor's rotation orientations //
    m_talonLeftLead.setInverted(InvertType.None);
    m_talonRightLead.setInverted(InvertType.InvertMotorOutput);
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
