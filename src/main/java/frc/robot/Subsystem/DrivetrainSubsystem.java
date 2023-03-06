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
  SwerveModule m_f = new SwerveModule(SWERVE.SDFront);
  SwerveModule m_br = new SwerveModule(SWERVE.SDBackRight);
  SwerveModule m_bl = new SwerveModule(SWERVE.SDBackLeft);
  
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
        m_f.getPosition(),
        m_br.getPosition(),
        m_bl.getPosition()
        
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
    m_f.setDesiredState(swerveModuleStates[0],_optimize, false, false);
    m_br.setDesiredState(swerveModuleStates[1],_optimize, false, false);
    m_bl.setDesiredState(swerveModuleStates[2],_optimize, false, false);
    
  }
  // Line up the wheels with the direction requested from x,y.
  // Drive to the distance
  public void driveAuto(double _xSpeed, double _ySpeed, double _rot,  boolean _optimize){
    
    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kinematics.toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_f.setDesiredState(swerveModuleStates[0],_optimize, false, false);
    m_br.setDesiredState(swerveModuleStates[1],_optimize, false, false);
    m_bl.setDesiredState(swerveModuleStates[2],_optimize, false, false);
    
  }

  public void steerAuto(double _xSpeed, double _ySpeed){
    SwerveModuleState[] swerveModuleStates = DRIVETRAIN.kinematics.toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVETRAIN.maxSpeed);

    m_f.setDesiredState(swerveModuleStates[0],false, true, false);
    m_br.setDesiredState(swerveModuleStates[1],false, true, false);
    m_bl.setDesiredState(swerveModuleStates[2],false, true, false);
    

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
    double dis = (m_br.getDriveDistanceMeters() + m_bl.getDriveDistanceMeters() + m_f.getDriveDistanceMeters())/3.0;
    return dis;
  }
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() * 39.37;
  }
  public double getRobotAngle(){
    double ang = 0;
    if(currentGyro == GyroEnum.AHRS){
      ang = m_gyro.getAngle();
    }else{
      ang = m_PGyro.getYaw();
    }

    return -ang;
  }

  public Rotation2d getRobotRotation2D(){
    double angle = getRobotAngle();
    return Rotation2d.fromDegrees(angle);
  }
  public void resetGyro(){
    m_gyro.reset();
    m_PGyro.setYaw(0);
  }
  public void resetDriveEncoders(){
    m_f.resetDriveEncoders();
    m_br.resetDriveEncoders();
    m_bl.resetDriveEncoders();
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
          m_f.getPosition(),
          m_br.getPosition(),
          m_bl.getPosition()
          
        });
  }
  public void setFieldRelative(boolean _frm){
    isFieldRelative = _frm;
  }
  public void resetSteerEncoders(){
    m_f.resetSteerSensors();
    m_bl.resetSteerSensors();
    m_br.resetSteerSensors();
  }
  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("DriveInches", getDriveDistanceInches());
    SmartDashboard.putNumber("PigeonAngle", -m_PGyro.getYaw());
    SmartDashboard.putNumber("NavXAngle", -m_gyro.getAngle());
    SmartDashboard.putNumber("RobotAngle", getRobotAngle());
    m_f.sendData();
    m_br.sendData();
    m_bl.sendData();
  }
}
