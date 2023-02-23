// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lib.ArmController;
import frc.robot.Lib.ArmPosEnum;

/**
 * The Arm consists of a Shoulder, Elbow and Intake 
 * 2 Shoulder motors working in parallel
 * 2 Elbow motors working in parallel
 * 1 Intake Spin motor
 * 1 Intake Rotate motor
 * 
 * To achieve a smooth movement the arm rotate motors must move at a velocity.
 * Option 1: 
 * Use a PID + Feedforward in voltage mode
 * An adjustment for the weight will need to be done. As the weight or angle changes the voltage does to maintain the position
 * The PID will fix the position and FeedForward will make it smooth. Hopefully
 * 
 * Option 2:
 * Profilled PID controls the velocity and acceleration.
 */
public class Arm extends SubsystemBase {
  public CANSparkMax m_leftShoulderMotCtrl;
  public CANSparkMax m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_elbowMotCtrl;
  public WPI_TalonSRX m_intakeRotateMotCtrl;
  public WPI_TalonSRX m_intakeSpinnerMotCtrl;
  public ArmController m_armController;

  


  /** Creates a new Arm. */
  public Arm() {
    m_leftShoulderMotCtrl = new CANSparkMax(25, MotorType.kBrushless);
    m_rightShoulderMotCtrl = new CANSparkMax(26, MotorType.kBrushless);
    m_leftShoulderMotCtrl.restoreFactoryDefaults();
    m_rightShoulderMotCtrl.restoreFactoryDefaults();
    m_rightShoulderMotCtrl.follow(m_leftShoulderMotCtrl,true);


  }
  public void moveShoulder(double _volts){
    m_leftShoulderMotCtrl.setVoltage(_volts);
  }
  public void moveElbow(double _volts){
    m_elbowMotCtrl.setVoltage(_volts);
  }
  public void moveHand(double _volts){
    m_intakeRotateMotCtrl.setVoltage(_volts);
  }
  public void spinHand(double _volts){
    m_intakeSpinnerMotCtrl.setVoltage(_volts);
  }
  public void moveToPos(ArmPosEnum _pos){
    m_armController.moveToPosition(_pos);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
