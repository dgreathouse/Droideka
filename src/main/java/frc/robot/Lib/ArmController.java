// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Subsystem.ArmSubsystem;

/** Arm Controller
 * kV controls the feedforward gain for velocity
 * Or
 * PID controls the Velocity with max output being the set velocity
 */
public class ArmController {

    PIDController pid;
    double kS;
    double kG;
    double kV;
    double kP;
    double kI;
    double maxVelocity = 0;
    ArmSubsystem arm;

    /**
     * 
     * @param kS Units (Volts) static value to overcome friction
     * @param kG Units (Volts) Gravity gain to maintain postion at angle
     * @param kV Units (Radians/Sec)
     */
    public ArmController(double _kS, double _kG, double _kV, double _kP, double _kI, ArmSubsystem _arm){
        this.kS = _kS;
        this.kG = _kG;
        this.kV = _kV;
        this.kP = _kP;
        this.kI = _kI;
        arm = _arm;
        pid = new PIDController(kP, kI, 0);
    }
    public double calculate(double _position, double _velocity, double _elbowPosition, double _intakeAngle){
        maxVelocity = _velocity;
        // getVelocity of motor
        // get Angle of arm segments
        //
        // PID calculate based on position
        // Max PID is _Velocity
        // Set velocity must become 0 at target because position error is 0
        return 0;
    }
    public void setKS(double _k){
        kS = _k;
    }
    public void setKG(double _k){
        kG = _k;
    }
    public void setKV(double _k){
        kV = _k;
    }
}
