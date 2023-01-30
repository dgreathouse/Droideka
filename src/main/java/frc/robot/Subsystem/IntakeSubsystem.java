// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k.INTAKE;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_spinMot;
  private WPI_TalonSRX m_rotateMot;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //m_spinMot = new WPI_TalonSRX(INTAKE.spinMotCANId);
    //m_rotateMot = new WPI_TalonSRX(INTAKE.rotateMotCANId);

  }
  public void spin(double _speed){
    //m_spinMot.set(_speed);
  }
  public void rotate(double _angle){
    //m_rotateMot.set(ControlMode.Position, _angle*INTAKE.cntsPDeg);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
