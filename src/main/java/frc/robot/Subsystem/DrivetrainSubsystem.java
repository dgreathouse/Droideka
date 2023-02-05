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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lib.GyroEnum;
import frc.robot.Lib.RotationMode;
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
  RotationMode m_rotationMode = RotationMode.AxisSpeed;
  public boolean isFieldRelative = true;
  double rotationPIDAngle = 0;
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
  // public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelativeMode){
  //   isFieldRelative = _fieldRelativeMode;
  //   drive(_xSpeed,_ySpeed,_rot);
  // }
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _isFieldRelative, boolean _optimize){
    isFieldRelative = _isFieldRelative;
    var swerveModuleStates =
        DRIVETRAIN.kinematics.toSwerveModuleStates(
          isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, getRobotRotation2D())
            : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_fl.setDesiredState(swerveModuleStates[0],_optimize, false, false);
    m_fr.setDesiredState(swerveModuleStates[1],_optimize, false, false);
    m_b.setDesiredState(swerveModuleStates[2],_optimize, false, false);
  }
  // Line up the wheels with the direction requested from x,y.
  // Drive to the distance
  public void driveAuto(double _xSpeed, double _ySpeed, double _rot,  boolean _optimize){
    
    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kinematics.toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_fl.setDesiredState(swerveModuleStates[0],_optimize, false, false);
    m_fr.setDesiredState(swerveModuleStates[1],_optimize, false, false);
    m_b.setDesiredState(swerveModuleStates[2],_optimize, false, false);
  }

  public void steerAuto(double _xSpeed, double _ySpeed){
    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kinematics.toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_fl.setDesiredState(swerveModuleStates[0],false, true, false);
    m_fr.setDesiredState(swerveModuleStates[1],false, true, false);
    m_b.setDesiredState(swerveModuleStates[2],false, true, false);

  }
  public void setRotationPIDAngle(double _val) {
    rotationPIDAngle = _val;
  }
  public double getRotationPIDAngle(){
    return rotationPIDAngle;
  }
  /**
   * 
   * @return Distance in Inches
   */
  public double getDriveDistanceMeters(){
    double dis = (m_fl.getDriveDistanceMeters() + m_fr.getDriveDistanceMeters() + m_b.getDriveDistanceMeters())/3.0;
    return dis;
  }
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() * 39.37;
  }
  public double getRobotAngle(){
    if(currentGyro == GyroEnum.AHRS){
      return m_gyro.getAngle();
    }else if(currentGyro == GyroEnum.PIGEON2){
      return m_PGyro.getYaw();
    }
    return m_gyro.getAngle();
  }
  public double getRobotPitch(){
    if(currentGyro == GyroEnum.AHRS){
      return m_gyro.getPitch();
    }else if(currentGyro == GyroEnum.PIGEON2){
      return m_PGyro.getPitch();
    }
    return m_gyro.getPitch();
  }
  public Rotation2d getRobotRotation2D(){
    if(currentGyro == GyroEnum.AHRS){
      return m_gyro.getRotation2d();
    }else if(currentGyro == GyroEnum.PIGEON2){
      Rotation2d r2d = new Rotation2d(m_PGyro.getYaw());
      return r2d;
    }
    return m_gyro.getRotation2d();
  }
  public void resetGyro(){
    m_gyro.reset();
    m_PGyro.setYaw(0);
  }
  public void resetDriveEncoders(){
    m_b.resetDriveEncoders();
    m_fl.resetDriveEncoders();
    m_fr.resetDriveEncoders();
  }
  public void switchGyro(){
    if(currentGyro == GyroEnum.AHRS){
      currentGyro = GyroEnum.PIGEON2;
    }else if(currentGyro == GyroEnum.PIGEON2){
      currentGyro = GyroEnum.AHRS;
    }
  }
  public void switchRotationMode(){
    if(m_rotationMode == RotationMode.PIDAngle){
      m_rotationMode = RotationMode.AxisSpeed;
    }else if(m_rotationMode == RotationMode.AxisSpeed){
      m_rotationMode = RotationMode.PIDAngle;
    }
  }
  public RotationMode getRotationMode(){
    return m_rotationMode;
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
  public void setFieldRelative(boolean _frm){
    isFieldRelative = _frm;
  }
  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putNumber("DriveInches", getDriveDistanceInches());
    SmartDashboard.putNumber("PigeonAngle", -m_PGyro.getYaw());
    SmartDashboard.putNumber("NavXAngle", m_gyro.getAngle());
    m_b.sendData();
    m_fl.sendData();
    m_fr.sendData();
  }
}
