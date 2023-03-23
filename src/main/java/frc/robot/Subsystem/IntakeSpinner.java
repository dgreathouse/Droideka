// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;

public class IntakeSpinner extends SubsystemBase {
  /** Creates a new IntakeSpinner. */
  public WPI_TalonSRX m_intakeSpinnerMotCtrl;
  public IntakeSpinner() {
   m_intakeSpinnerMotCtrl = new WPI_TalonSRX(k.INTAKE.spinCANID);
   m_intakeSpinnerMotCtrl.configFactoryDefault();
   //m_intakeSpinnerMotCtrl.setOpenLoopRampRate(1);
   m_intakeSpinnerMotCtrl.setNeutralMode(NeutralMode.Brake);
  }
  public void spinHand(double _speed){
    double speed = _speed;
    double volts = speed * 12;

    m_intakeSpinnerMotCtrl.setVoltage(volts);
  }


  public double getIntakeCurrent(){
    return 0;
  // return 0;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeCurrent", getIntakeCurrent());
    
  }
}
