// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
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
  public static double kElbowDegPerCnt = 0.080142;
  public static double kHandDegPerCnt = 90/34.73;
  public CANSparkMax m_leftShoulderMotCtrl;
  public CANSparkMax m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_leftElbowMotCtrl;
  public WPI_TalonSRX m_rightElbowMotCtrl;
  public CANSparkMax m_intakeRotateMotCtrl;
  
  public ArmController m_armController;

  
  /** Creates a new Arm. */
  public Arm() {
    // m_leftShoulderMotCtrl = new CANSparkMax(k.SHOULDER.leftCANId, MotorType.kBrushless);
    // m_rightShoulderMotCtrl = new CANSparkMax(k.SHOULDER.rightCANId, MotorType.kBrushless);
    // m_leftShoulderMotCtrl.restoreFactoryDefaults();
    // m_rightShoulderMotCtrl.restoreFactoryDefaults();
    // m_rightShoulderMotCtrl.follow(m_leftShoulderMotCtrl,true);
    // m_rightShoulderMotCtrl.getEncoder().setPosition(0);
    // m_leftShoulderMotCtrl.getEncoder().setPosition(0);

    // m_leftElbowMotCtrl = new WPI_TalonSRX(k.ELBOW.leftCANId);
    // m_leftElbowMotCtrl.configFactoryDefault();
    // m_leftElbowMotCtrl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // m_leftElbowMotCtrl.setNeutralMode(NeutralMode.Brake);
    // m_leftElbowMotCtrl.setSelectedSensorPosition(0);

    // m_rightElbowMotCtrl = new WPI_TalonSRX(k.ELBOW.rightCANId);
    // m_rightElbowMotCtrl.configFactoryDefault();
    // m_rightElbowMotCtrl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // m_rightElbowMotCtrl.setNeutralMode(NeutralMode.Brake);
    // m_rightElbowMotCtrl.setSelectedSensorPosition(0);
    // m_rightElbowMotCtrl.follow(m_leftElbowMotCtrl);
    

    // m_intakeRotateMotCtrl = new CANSparkMax(k.INTAKE.rigthSpinCANID, MotorType.kBrushless);

    // m_intakeRotateMotCtrl.restoreFactoryDefaults();
    // m_intakeRotateMotCtrl.setIdleMode(IdleMode.kCoast);

    

    // m_rightShoulderMotCtrl.getEncoder().setPosition(0.0);
    // m_armController = new ArmController(this);

  }

  public double getShoulderAngle(){
    return 0;
  //  return m_leftShoulderMotCtrl.getEncoder().getPosition()* kShoulderDegPerCnt;
  }
  public double getElbowAngle(){
    return 0;
  //  return m_leftElbowMotCtrl.getSelectedSensorPosition() * kElbowDegPerCnt;
  }
  public double getHandAngle(){
  return 0;
  //  return m_intakeRotateMotCtrl.getEncoder().getPosition() * kHandDegPerCnt;
  }
  
  public void moveShoulder(double _volts){
   //// m_leftShoulderMotCtrl.setVoltage(_volts);
  }
  public void moveElbow(double _volts){
   /////// m_leftElbowMotCtrl.setVoltage(_volts);
  }
  public void moveHand(double _volts){
  //  m_intakeRotateMotCtrl.setVoltage(_volts);
  }

  public void setArmPos(ArmPosEnum _pos){
   // m_armController.m_armPos = _pos;
  }

  @Override
  public void periodic() {
    
    // SmartDashboard.putNumber("SHMotCnts", getShoulderAngle());
    // SmartDashboard.putNumber("ELMotCnts", getElbowAngle());
    // SmartDashboard.putNumber("HAAngle", getHandAngle());

    // SmartDashboard.putString("Arm Pos", m_armController.m_armPos.toString());
  }
}
