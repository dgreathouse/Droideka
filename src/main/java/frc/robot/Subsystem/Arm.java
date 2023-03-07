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
import frc.robot.RobotContainer;
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
  public static double kElbowDegPerCnt = .080142;
  public static double kHandDegPerCnt = 90/8.59;
  public CANSparkMax m_leftShoulderMotCtrl;
  public CANSparkMax m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_leftElbowMotCtrl;
  public WPI_TalonSRX m_rightElbowMotCtrl;
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
    m_rightShoulderMotCtrl.getEncoder().setPosition(0);
    m_leftShoulderMotCtrl.getEncoder().setPosition(0);
   
    m_leftElbowMotCtrl = new WPI_TalonSRX(48);
    m_leftElbowMotCtrl.configFactoryDefault();
    m_leftElbowMotCtrl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_leftElbowMotCtrl.setNeutralMode(NeutralMode.Brake);
    m_leftElbowMotCtrl.setSelectedSensorPosition(0);

    m_rightElbowMotCtrl = new WPI_TalonSRX(49);
    m_rightElbowMotCtrl.configFactoryDefault();
    m_rightElbowMotCtrl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightElbowMotCtrl.setNeutralMode(NeutralMode.Brake);
    m_rightElbowMotCtrl.setSelectedSensorPosition(0);
    m_rightElbowMotCtrl.follow(m_leftElbowMotCtrl);

    m_intakeRotateMotCtrl = new CANSparkMax(27, MotorType.kBrushless);
    m_intakeSpinnerMotCtrl = new CANSparkMax(28, MotorType.kBrushless);
    m_intakeRotateMotCtrl.restoreFactoryDefaults();
    m_intakeRotateMotCtrl.setIdleMode(IdleMode.kBrake);
    m_intakeSpinnerMotCtrl.setIdleMode(IdleMode.kBrake);
    m_rightShoulderMotCtrl.getEncoder().setPosition(0.0);
    m_armController = new ArmController(this);

  }
  public void moveShoulder(double _volts){
    m_leftShoulderMotCtrl.setVoltage(_volts);
  }
  public double getShoulderAngle(){
    return m_leftShoulderMotCtrl.getEncoder().getPosition()* kShoulderDegPerCnt;
  // return 0;
  }
  public void moveElbow(double _volts){
    m_leftElbowMotCtrl.setVoltage(_volts);

  }
  public double getElbowAngle(){
    return m_leftElbowMotCtrl.getSelectedSensorPosition() * kElbowDegPerCnt;
    //return 0;
  }
  public void moveHand(double _volts){
    m_intakeRotateMotCtrl.setVoltage(_volts);
  }
  
  public void spinHand(double _speed){
    double speed = _speed;
    // if(m_intakeSpinnerMotCtrl.getOutputCurrent() > 10){
    //   speed = 0;
    // }
    double volts = speed * 12;
    SmartDashboard.putNumber("Spinner Volts", volts);
    m_intakeSpinnerMotCtrl.setVoltage(volts);
  }
  public double getHandAngle(){
    return m_intakeRotateMotCtrl.getEncoder().getPosition() * kHandDegPerCnt;
   //return 0;
  }
  public void setArmPos(ArmPosEnum _pos){
    m_armController.m_armPos = _pos;
  }
  public double getIntakeCurrent(){
    return m_intakeSpinnerMotCtrl.getOutputCurrent();
    
   // return RobotContainer.pd.getCurrent(5);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spinner Current", getIntakeCurrent());
    SmartDashboard.putNumber("SHMotCnts", getShoulderAngle());
    SmartDashboard.putNumber("ELMotCnts", getElbowAngle());
    SmartDashboard.putNumber("HAAngle", getHandAngle());
    // SmartDashboard.putString("Arm Pos", m_armController.m_armPos.toString());

    // This method will be called once per scheduler run
  }
}
