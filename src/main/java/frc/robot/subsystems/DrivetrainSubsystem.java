// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
  MotorControllerGroup m_left = new MotorControllerGroup(m_talonLeftLead, m_talonLeftFollow);
  MotorControllerGroup m_right = new MotorControllerGroup(m_talonRightLead, m_talonRightFollow);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  // Gyro //
  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(Constants.PIGEON2_ID);

  // Odometry class for tracking robot pose //
  private final DifferentialDriveOdometry m_odometry;

  NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  NetworkTableEntry ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");

  // private final SimpleMotorFeedforward  m_tankDriveVoltsFeedforward;

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

    // Set neutral mode to coast on all motors //
    m_talonLeftLead.setNeutralMode(NeutralMode.Coast);
    m_talonLeftFollow.setNeutralMode(NeutralMode.Coast);
    m_talonRightLead.setNeutralMode(NeutralMode.Coast);
    m_talonRightFollow.setNeutralMode(NeutralMode.Coast);

    // Set Falcon 500 Voltage Compensation to 10V //
    m_talonLeftLead.configVoltageCompSaturation(10);
    m_talonLeftFollow.configVoltageCompSaturation(10);
    m_talonRightLead.configVoltageCompSaturation(10);
    m_talonRightFollow.configVoltageCompSaturation(10);
    
    m_talonLeftLead.configOpenloopRamp(0.01);
    m_talonLeftFollow.configOpenloopRamp(0.01);
    m_talonRightLead.configOpenloopRamp(0.01);
    m_talonRightLead.configOpenloopRamp(0.01);

    // Set our followers to follow the lead motor //
    // m_talonLeftFollow.follow(m_talonLeftLead);
    // m_talonRightFollow.follow(m_talonRightLead);

    // Set our follower's inverted to be opposite of the master //
    m_talonLeftFollow.setInverted(TalonFXInvertType.CounterClockwise);
    m_talonRightFollow.setInverted(TalonFXInvertType.Clockwise);

    // Set our lead motor's rotation orientations //
    m_talonLeftLead.setInverted(TalonFXInvertType.CounterClockwise);
    m_talonRightLead.setInverted(TalonFXInvertType.Clockwise);

    // Configure encoder readings on the TalonFX //
    m_talonLeftLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_talonRightLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // Reset encoders to 0 //
    resetEncoders();
    
    // Intialize all gyro readings to 0 //
    zeroHeading();

    // Initialize Robot Odometry //
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      getDistance(m_talonLeftLead),
      getDistance(m_talonRightLead)
    );
  }

  public void setNeutralMode(NeutralMode mode) {
    m_talonLeftLead.setNeutralMode(mode);
    m_talonLeftFollow.setNeutralMode(mode);
    m_talonRightLead.setNeutralMode(mode);
    m_talonRightFollow.setNeutralMode(mode);
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_talonLeftLead.setVoltage(leftVolts);   // Set voltage for left motor
    m_talonLeftFollow.setVoltage(leftVolts);
    m_talonRightLead.setVoltage(rightVolts);  // Set voltage for right motor
    m_talonRightFollow.setVoltage(rightVolts);
    m_drive.feed();                   // Feed the motor safety object, stops the motor if anything goes wrong
  }

  public void tankDriveVoltsAuto(double leftVolts, double rightVolts) {
    // Put a hard limit on the voltage to make sure we're going as slow as possible //
    if (leftVolts > 1.5) {
      leftVolts = 1.5;
    } else if (leftVolts < -1.5) {
      leftVolts = -1.5;
    }

    if (rightVolts > 1.5) {
      rightVolts = 1.5;
    } else if (rightVolts < -1.5) {
      rightVolts = -1.5;
    }

    m_talonLeftLead.setVoltage(leftVolts);   // Set voltage for left motor
    m_talonLeftFollow.setVoltage(leftVolts);
    m_talonRightLead.setVoltage(rightVolts);  // Set voltage for right motor
    m_talonRightFollow.setVoltage(rightVolts);    m_drive.feed();                   // Feed the motor safety object, stops the motor if anything goes wrong

  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // Encoder Controls/Readings //
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    double leftSpeed = m_talonLeftLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;
    double rightSpeed = m_talonRightLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;  // Need to invert the results
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  public void resetEncoders(){
    m_talonLeftLead.setSelectedSensorPosition(0.0, 0, 0);
    m_talonRightLead.setSelectedSensorPosition(0.0, 0, 0);
  }

  public double getDistance(WPI_TalonFX motor){
    return motor.getSelectedSensorPosition() * Constants.ENCODER_DISTANCE_PER_PULSE;
  }

  // Gets the average encoder distance in meters //
  public double getAverageEncoderDistance() {
    return (getDistance(m_talonLeftLead) + getDistance(m_talonRightLead)) / 2.0;
  }

  // Gyro Controls/Readings //
  
  /** Zeroes the heading of the robot. */
  public void zeroHeading(){
    m_gyro.reset();   // Resets the Gyro
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getPitch() {
    System.out.println(m_gyro.getPitch());
    return m_gyro.getPitch();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(m_gyro.getRotation2d(), getDistance(m_talonLeftLead), getDistance(m_talonRightLead), pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), getDistance(m_talonLeftLead), getDistance(m_talonRightLead));

    SmartDashboard.putNumber("Pigeon 2 Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Pigeon 2 Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Pigeon 2 Yaw", m_gyro.getYaw());

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
    SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));

    SmartDashboard.putNumber("FalconLeftTopTemp", m_talonLeftLead.getTemperature());
    SmartDashboard.putNumber("FalconLeftFollowTemp", m_talonLeftFollow.getTemperature());
    SmartDashboard.putNumber("FalconRightTopTemp", m_talonRightLead.getTemperature());
    SmartDashboard.putNumber("FalconRightFollowTemp", m_talonRightFollow.getTemperature());
    

    // double degree = getHeading();
    // m_angleEntry.setDouble(degree);

    // // Output raw encoder values //
    // m_leftEncoderEntry.setDouble(m_talonLeftLead.getSelectedSensorPosition());
    // m_rightEncoderEntry.setDouble(m_talonRightLead.getSelectedSensorPosition());

    // // Output encoder values converted to distance //
    // m_leftEncoderDistanceEntry.setDouble(getDistance(m_talonLeftLead));
    // m_rightEncoderDistanceEntry.setDouble(getDistance(m_talonRightLead));

    // // Output raw encoder velocity values //
    // m_leftEncoderVelocityEntry.setDouble(m_talonLeftLead.getSelectedSensorVelocity());
    // m_rightEncoderVelocityEntry.setDouble(m_talonRightLead.getSelectedSensorVelocity());

    // // Output raw wheel speed values //
    // m_leftEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().leftMetersPerSecond);
    // m_rightEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().rightMetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
