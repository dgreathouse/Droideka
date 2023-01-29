// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lib.GyroEnum;
import frc.robot.Lib.SwerveModule;
import frc.robot.k.DRIVETRAIN;
import frc.robot.k.SWERVE;

public class DrivetrainSubsystem extends SubsystemBase {
  SwerveModule m_fl = new SwerveModule(SWERVE.SDFrontLeft);
  SwerveModule m_fr = new SwerveModule(SWERVE.SDFrontRight);
  SwerveModule m_b = new SwerveModule(SWERVE.SDBack);
  GyroEnum currentGyro = GyroEnum.AHRS;
  AHRS m_gyro = new AHRS(Port.kMXP);
  Pigeon2 m_PGyro = new Pigeon2(5);
  private final SwerveDriveOdometry m_odometry = 
    new SwerveDriveOdometry(
      DRIVETRAIN.kinematics, getRobotRotation2D(), 
      new SwerveModulePosition[]{
        m_fl.getPosition(),
        m_fr.getPosition(),
        m_b.getPosition()
      });
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    resetGyro();
    
  }
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative){
    var swerveModuleStates =
        DRIVETRAIN.kinematics.toSwerveModuleStates(
            _fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, getRobotRotation2D())
                : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_fl.setDesiredState(swerveModuleStates[0]);
    m_fr.setDesiredState(swerveModuleStates[1]);
    m_b.setDesiredState(swerveModuleStates[2]);
  }
  /**
   * 
   * @return Distance in Meters
   */
  public double getDriveDistance(){
    double dis = (m_fl.getDriveDistance() + m_fr.getDriveDistance() + m_b.getDriveDistance())/3.0;
    return dis;
  }
  public double getRobotAngle(){
    return m_gyro.getAngle();
  }
  public Rotation2d getRobotRotation2D(){
    return m_gyro.getRotation2d();
  }
  public void resetGyro(){
    m_gyro.reset();
  }
  public void switchGyro(){
    if(currentGyro == GyroEnum.AHRS){
      currentGyro = GyroEnum.PIGEON2;
    }else if(currentGyro == GyroEnum.PIGEON2){
      currentGyro = GyroEnum.AHRS;
    }
  }
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_fl.getPosition(),
          m_fr.getPosition(),
          m_b.getPosition()
        });
  }
  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putNumber("PigeonAngle", m_PGyro.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("NavXAngle", m_gyro.getAngle());
    m_b.sendData();
    m_fl.sendData();
    m_fr.sendData();
  }
}
