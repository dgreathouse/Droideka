// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;

public class IntakeSpinner extends SubsystemBase {
  /** Creates a new IntakeSpinner. */
 // public CANSparkMax m_intakeSpinnerMotCtrl;
  public IntakeSpinner() {
 //   m_intakeSpinnerMotCtrl = new CANSparkMax(k.INTAKE.leftRotateCANID, MotorType.kBrushless);
    //m_intakeSpinnerMotCtrl.setOpenLoopRampRate(1);
  //  m_intakeSpinnerMotCtrl.setIdleMode(IdleMode.kBrake);
  }
  public void spinHand(double _speed){
    double speed = _speed;
    double volts = speed * 12;

   // m_intakeSpinnerMotCtrl.setVoltage(volts);
  }
  public double getSpinnerCnts(){
    return 0;
   // return m_intakeSpinnerMotCtrl.getEncoder().getPosition();
  }
  public void setSpinnerCnts(double val){
   
  //  m_intakeSpinnerMotCtrl.getEncoder().setPosition(0);
  }
  public double getIntakeCurrent(){
   // return m_intakeSpinnerMotCtrl.getOutputCurrent();/
   return 0;
    
   // return RobotContainer.pd.getCurrent(5);
  }
  @Override
  public void periodic() {
 //   SmartDashboard.putNumber("IntakeCurrent", getIntakeCurrent());
    
  }
}
