// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
import frc.robot.Lib.GyroEnum;

import frc.robot.Lib.SwerveModule;
import frc.robot.k.DRIVETRAIN;
import frc.robot.k.SWERVE;

public class DrivetrainSubsystem extends SubsystemBase {
  SwerveModule m_b = new SwerveModule(SWERVE.SDBack);
  SwerveModule m_fl = new SwerveModule(SWERVE.SDFrontLeft);
  SwerveModule m_fr = new SwerveModule(SWERVE.SDFrontRight);
  
  GyroEnum currentGyro = GyroEnum.PIGEON2;
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public NeutralMode driveNeutralMode = NeutralMode.Brake;
  Pigeon2 m_PGyro = new Pigeon2(5);
 // RotationMode m_rotationMode = RotationMode.AxisSpeed;
  public boolean isFieldRelative = true;
 // double rotationPIDAngle = 0;
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

  /**
   * 
   * @return Distance in Inches
   */
  public double getDriveDistanceMeters(){
    double dis = (m_fl.getDriveDistanceMeters() + m_fr.getDriveDistanceMeters() + m_b.getDriveDistanceMeters())/3.0;
    return dis;
  }
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / k.DRIVETRAIN.MetersPerInch;
  }
  public double getRobotAngle(){
    double ang = 0;
    if(currentGyro == GyroEnum.PIGEON2){
      ang = m_PGyro.getYaw();
      
    }else{
      ang = m_gyro.getAngle();
    }

    return ang;
  }

  public Rotation2d getRobotRotation2D(){
    double angle = getRobotAngle();
    return Rotation2d.fromDegrees(angle);
  }
  public void resetGyro(){
    m_gyro.reset();
    m_gyro.calibrate();
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
  // public void switchRotationMode(){
  //   if(m_rotationMode == RotationMode.PIDAngle){
  //     m_rotationMode = RotationMode.AxisSpeed;
  //   }else if(m_rotationMode == RotationMode.AxisSpeed){
  //     m_rotationMode = RotationMode.PIDAngle;
  //   }
  // }
  // public RotationMode getRotationMode(){
  //   return m_rotationMode;
  // }
  public void updateOdometry() {
    m_odometry.update(
      Rotation2d.fromDegrees(getRobotAngle()),
        new SwerveModulePosition[] {
          
          m_fl.getPosition(),
          m_fr.getPosition(),
          m_b.getPosition()
          
        });
  }
  public void setFieldRelative(boolean _frm){
    isFieldRelative = _frm;
  }
  public void resetSteerEncoders(){
    m_b.resetSteerSensors();
    m_fr.resetSteerSensors();
    m_fl.resetSteerSensors();
  }
  public void setDriveNeutralMode(NeutralMode _mode){
    m_b.setDriveNeutralMode(_mode);
    m_fr.setDriveNeutralMode(_mode);
    m_fl.setDriveNeutralMode(_mode);

  }
  public void switchDriveNeutralMode(){
    if(driveNeutralMode == NeutralMode.Brake){
      driveNeutralMode = NeutralMode.Coast;
    }else {
      driveNeutralMode = NeutralMode.Brake;
    }
    setDriveNeutralMode(driveNeutralMode);
  }
  @Override
  public void periodic() {
    updateOdometry();

    // SmartDashboard.putNumber("DriveInches", getDriveDistanceInches());
    // SmartDashboard.putNumber("PigeonAngle", -m_PGyro.getYaw());
    // SmartDashboard.putNumber("NavXAngle", -m_gyro.getAngle());
     SmartDashboard.putNumber("RobotAngle", getRobotAngle());
     SmartDashboard.putString("Current Gyro", currentGyro.toString());
     SmartDashboard.putBoolean("Relative ", isFieldRelative);
    m_b.sendData();
    m_fl.sendData();
    m_fr.sendData();
  }
}
