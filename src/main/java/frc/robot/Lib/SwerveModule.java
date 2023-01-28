// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.k;


/** Add your docs here. */
public class SwerveModule {

    WPI_TalonFX m_driveFx;
    WPI_TalonFX m_steerFx;
    CANCoder m_steerEnc;
    SwerveData m_data;
   
    private PIDController m_drivePidController = new PIDController(k.SWERVE.driveKp, k.SWERVE.driveKi, k.SWERVE.driveKd);
    private ProfiledPIDController m_steerPIDController = 
        new ProfiledPIDController(k.SWERVE.steerKp, k.SWERVE.steerKi, k.SWERVE.steerKd, 
        new TrapezoidProfile.Constraints(k.SWERVE.steerMax_RadPS/2, k.SWERVE.steerMax_RadPSSq/2));

    private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(k.SWERVE.driveSMFKs, k.SWERVE.driveSMFKv, k.SWERVE.driveSMFKa);
    private SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(k.SWERVE.steerSMFKs, k.SWERVE.steerSMFKv,k.SWERVE.steerSMFKa);

    public SwerveModule(SwerveData _data){
        m_driveFx = new WPI_TalonFX(_data.driveCANID);
        m_steerFx = new WPI_TalonFX(_data.steerCANID);
        m_steerEnc = new CANCoder(_data.canCoderCANID);
        m_data = _data;

        
        m_driveFx.configVoltageCompSaturation(12.0, 20);
        m_driveFx.enableVoltageCompensation(true);

        m_steerFx.configVoltageCompSaturation(12.0, 20);
        m_steerFx.enableVoltageCompensation(true);

        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_steerEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    }
    
    /**
     * 
     * @return Disitance in Meters
     */
    public double getDriveDistance(){
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
    public double getSteerAngle(){
        return m_steerFx.getSelectedSensorPosition() / k.SWERVE.steer_CntsPRad;
        
    }
        /**
     * 
     * @return Angle in Radians of steer motor
     */
    public double getSwerveAngle(){
        return Math.toRadians(m_steerEnc.getAbsolutePosition());
    }
   
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getSteerAngle()));
    }
    public void setDesiredState(SwerveModuleState _desiredState){
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(_desiredState, new Rotation2d(getSteerAngle()));
        // Calculate the drive output from the drive PID controller.
        double driveOutput = m_drivePidController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double steerOutput = m_steerPIDController.calculate(getSteerAngle(), state.angle.getRadians());
        double steerFeedforward = m_steerFeedforward.calculate(m_steerPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber(m_data.name + "_SFF",steerFeedforward);
        SmartDashboard.putNumber(m_data.name + "_DFF",driveFeedforward);
        SmartDashboard.putNumber(m_data.name + "_SPIDOut",steerOutput);
        SmartDashboard.putNumber(m_data.name + "_DPIDOut",driveOutput);
        
        
        m_driveFx.setVoltage(driveOutput + driveFeedforward);
        m_steerFx.setVoltage(steerOutput + steerFeedforward);
    }
}
