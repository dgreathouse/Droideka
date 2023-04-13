// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.k;
import frc.robot.k.DRIVETRAIN;


/** Add your docs here. */
public class SwerveModule {

    WPI_TalonFX m_driveFx;
    WPI_TalonFX m_steerFx;
    CANCoder m_steerEnc;
    SwerveData m_data;
   


    private ProfiledPIDController m_steerPIDController = 
        new ProfiledPIDController(k.SWERVE.steerKp, k.SWERVE.steerKi, k.SWERVE.steerKd, 
        new TrapezoidProfile.Constraints(k.SWERVE.steerMax_RadPS*DRIVETRAIN.maxVoltage, k.SWERVE.steerMax_RadPSSq*DRIVETRAIN.maxVoltage));



    public SwerveModule(SwerveData _data){
        m_data = _data;
        m_driveFx = new WPI_TalonFX(_data.driveCANID);
        m_steerFx = new WPI_TalonFX(_data.steerCANID);
        m_steerEnc = new CANCoder(_data.canCoderCANID);
        
        m_steerEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        m_driveFx.setInverted(_data.driveInvert);
        m_steerFx.setInverted(_data.steerInvert);
        
        m_driveFx.configVoltageCompSaturation(DRIVETRAIN.maxVoltage);

        m_driveFx.enableVoltageCompensation(true);
        
        m_driveFx.setNeutralMode(NeutralMode.Coast);
        m_driveFx.setSelectedSensorPosition(0);

        m_steerFx.configVoltageCompSaturation(DRIVETRAIN.maxVoltage);

        m_steerFx.enableVoltageCompensation(true);
        m_steerFx.setNeutralMode(NeutralMode.Brake);

        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_steerPIDController.setTolerance(0.01);
    }
    public void resetDriveEncoders(){
        m_driveFx.setSelectedSensorPosition(0);
    }
    public void resetSteerSensors(){
        m_steerFx.setSelectedSensorPosition((Math.toDegrees(getSwerveAngle()) - m_data.angleOffset_Deg)*k.SWERVE.kSteerMotCntsPerWheelDeg);
    }
    /**
     * 
     * @return Disitance in Meters
     */
    public double getDriveDistanceMeters(){
        final double dis = m_driveFx.getSelectedSensorPosition();
        final double meters = dis / k.SWERVE.driveDistanceCntsPMeter;
        return meters;
    }

    public double getDriveVelocity(){
        // Get the FalconFX velocity in raw units /100 ms
        double vel1 = m_driveFx.getSelectedSensorVelocity();
        // Convert to Meters/s
        double velocity = vel1 / k.SWERVE.driveRawVelocityToMPS;
        return velocity;
    }
    public double getSteerMotorAngle(){
        return m_steerFx.getSelectedSensorPosition() / k.SWERVE.steer_CntsPRad;
        
    }
        /**
     * 
     * @return Angle in Radians of steer motor using CANCoder
     */
    public double getSwerveAngle(){
        return Math.toRadians(m_steerEnc.getAbsolutePosition());
    }
   
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), new Rotation2d(getSteerMotorAngle()));
    }
    public void setDesiredState(SwerveModuleState _desiredState, boolean _optimize, boolean _disableDrive, boolean _disableSteer){

        double steerOutput = 0;

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state;
        if(_optimize){
            state = SwerveModuleState.optimize(_desiredState, new Rotation2d(getSteerMotorAngle()));
        }else{
            state = _desiredState;
        }

        if(!_disableDrive){
           m_driveFx.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DRIVETRAIN.maxSpeed);
        }
        // Calculate the turning motor output from the turning PID controller.
        if(!_disableSteer){
            steerOutput = m_steerPIDController.calculate(getSteerMotorAngle(), state.angle.getRadians());
            m_steerFx.set(ControlMode.PercentOutput,steerOutput);
        }
        // SmartDashboard


        // SmartDashboard.putNumber(m_data.name + "StateSpeed", state.speedMetersPerSecond);
        // SmartDashboard.putNumber(m_data.name + "StateAngle", state.angle.getRadians());

        
    }
    public void setDriveNeutralMode(NeutralMode _mode){
        m_driveFx.setNeutralMode(_mode);
    }
    public void sendData(){
        SmartDashboard.putNumber(m_data.name+ "SteerMotorAngle", Math.toDegrees(getSteerMotorAngle()));
        SmartDashboard.putNumber(m_data.name+ "CANCoderAngle", Math.toDegrees(getSwerveAngle()));
        SmartDashboard.putNumber(m_data.name + "DriveCounts",getDriveDistanceMeters());
    }
}
