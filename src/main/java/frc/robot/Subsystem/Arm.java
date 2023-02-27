// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

 // 4096 * Rev = 124536 for 90Deg
  public static double kShoulderDegPerCnt = 90.0/30.53;
  public static double kElbowDegPerCnt = 90.0/110.0;
  public static double kHandDegPerCnt = 0;
  public CANSparkMax m_leftShoulderMotCtrl;
  public CANSparkMax m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_elbowMotCtrl;
  public CANSparkMax m_intakeRotateMotCtrl;
  public CANSparkMax m_intakeSpinnerMotCtrl;
  public ArmController m_armController;

  
  /** Creates a new Arm. */
  public Arm() {
    m_leftShoulderMotCtrl = new CANSparkMax(25, MotorType.kBrushless);
    m_rightShoulderMotCtrl = new CANSparkMax(26, MotorType.kBrushless);
    m_leftShoulderMotCtrl.restoreFactoryDefaults();
    m_rightShoulderMotCtrl.restoreFactoryDefaults();
    m_rightShoulderMotCtrl.follow(m_leftShoulderMotCtrl,true);
   
    m_elbowMotCtrl = new WPI_TalonSRX(48);
    m_elbowMotCtrl.configFactoryDefault();
    m_elbowMotCtrl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
   
    //m_intakeRotateMotCtrl = new CANSparkMax(27, MotorType.kBrushless);
    //m_intakeSpinnerMotCtrl = new CANSparkMax(28, MotorType.kBrushless);

    m_armController = new ArmController(this);

  }
  public void moveShoulder(double _volts){
    m_leftShoulderMotCtrl.setVoltage(_volts);
  }
  public double getShoulderAngle(){
    return m_leftShoulderMotCtrl.getEncoder().getPosition()* kShoulderDegPerCnt;
  }
  public void moveElbow(double _volts){
    m_elbowMotCtrl.setVoltage(-_volts);
  }
  public double getElbowAngle(){
    return m_elbowMotCtrl.getSelectedSensorPosition() * kElbowDegPerCnt;
  }
  public void moveHand(double _volts){
    m_intakeRotateMotCtrl.setVoltage(_volts);
  }
  public void spinHand(double _speed){
    m_intakeSpinnerMotCtrl.set(_speed);
  }
  public double getHandAngle(){
    return m_intakeRotateMotCtrl.getEncoder().getPosition() * kHandDegPerCnt;
  }
  public void setArmPos(ArmPosEnum _pos){
    m_armController.m_armPos = _pos;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SHMotCnts", getShoulderAngle());
    SmartDashboard.putNumber("ELMotCnts", getElbowAngle());
    SmartDashboard.putString("Arm Pos", m_armController.m_armPos.toString());

    // This method will be called once per scheduler run
  }
}
