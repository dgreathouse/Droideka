// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

/** Add your docs here. */
public class ArmData {
    double kS;
    double kG;
    double kV;
    double kP;
    double kI;
    double kD;
    double maxVelocity;
    double maxAccel;

    public ArmData(double _kS, double _kG, double _kV, double _kP, double _kI, double _maxVelocity, double _maxAccel)
    {
        kS = _kS;
        kG = _kG;
        kV = _kV;
        kP = _kP;
        kI = _kI;
        maxVelocity = _maxVelocity;
        maxAccel = _maxAccel;
    }
}
