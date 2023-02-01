// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
import frc.robot.Lib.ArmPosEnum;/**
 * The ArmSubsystem is a combination of many motors that control entire arm
 * Current naming is: 
 * Bicep that controls the upper arm
 * Elbow that controls the forearm or lower arm
 * Intake that controls the hand
 * Three (3) main position methods exist to move the Bicep,Elbow and Intake
 * The last method controls the intake velocity.
 */
public class ArmSubsystem extends SubsystemBase {
  public WPI_TalonFX m_leftShoulderMotCtrl;
  public WPI_TalonFX m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_leftElbowMotCtrl;
  public WPI_TalonSRX m_rightElbowMotCtrl;
  public WPI_TalonSRX m_intakeRotateMotCtrl;
  public WPI_TalonSRX m_intakeSpinnerMotCtrl;

  public ArmPosEnum m_armPos = ArmPosEnum.HOME;
  public double m_shoulderAngle = 0;
  public double m_elbowAngle = 0;
  public double m_intakeAngle = 0;
  public double m_intakeVelocity = 0;
  /**
   * 
   */
  /** Creates a new Arm. */
  public ArmSubsystem() {
    /************************ Shoulder ****************************/
    // Create new instance of motor control classes 
    m_leftShoulderMotCtrl = new WPI_TalonFX(k.ARM.leftShoulderCANId);
    m_rightShoulderMotCtrl = new WPI_TalonFX(k.ARM.rightShoulderCANId);

    // Invert the other controller that is going to follow
    m_rightShoulderMotCtrl.setInverted(InvertType.InvertMotorOutput);
    m_rightShoulderMotCtrl.follow(m_leftShoulderMotCtrl);

    // Configure the closedloop gain values
    m_leftShoulderMotCtrl.config_kP(0, 0.5);
    m_leftShoulderMotCtrl.config_kI(0, 0.0);
    m_leftShoulderMotCtrl.config_kD(0, 0.0);
    m_leftShoulderMotCtrl.configClosedLoopPeakOutput(0, 0.75);
    m_leftShoulderMotCtrl.configClosedloopRamp(1);

    /************************ ELBOW ****************************/
    // Create new instance of motor control classes 
    m_leftElbowMotCtrl = new WPI_TalonSRX(k.ARM.leftElbowCANId);
    m_rightElbowMotCtrl = new WPI_TalonSRX(k.ARM.rightElbowCANId);

    // Invert the other controller that is going to follow
    m_rightElbowMotCtrl.setInverted(InvertType.InvertMotorOutput);
    m_rightElbowMotCtrl.follow(m_leftShoulderMotCtrl);

    // Configure the closedloop gain values
    m_leftElbowMotCtrl.config_kP(0, 0.5);
    m_leftElbowMotCtrl.config_kI(0, 0.0);
    m_leftElbowMotCtrl.config_kD(0, 0.0);
    m_leftElbowMotCtrl.configClosedLoopPeakOutput(0, 0.75);
    m_leftElbowMotCtrl.configClosedloopRamp(1);

    /************************ INTAKE ****************************/
    // Create new instance of motor control classes 
    m_intakeRotateMotCtrl = new WPI_TalonSRX(k.ARM.leftIntakeRotateCANId);
    m_intakeSpinnerMotCtrl = new WPI_TalonSRX(k.ARM.rightIntakeSpinnerCANId);

    // Invert output if needed
    m_intakeRotateMotCtrl.setInverted(InvertType.None);
    

    // Configure the closedloop gain values
    m_intakeRotateMotCtrl.config_kP(0, 0.5);
    m_intakeRotateMotCtrl.config_kI(0, 0.0);
    m_intakeRotateMotCtrl.config_kD(0, 0.0);
    m_intakeRotateMotCtrl.configClosedLoopPeakOutput(0, 0.75);
    m_intakeRotateMotCtrl.configClosedloopRamp(1);

  }
  public void rotateShoulder(double _angle){
    double angle = _angle * k.ARM.shoulderCntsPDeg;
    m_leftShoulderMotCtrl.set(ControlMode.Position, angle);

  }
  public void rotateElbow(double _angle){
    double angle = _angle * k.ARM.elbowCntsPDeg;
    m_leftElbowMotCtrl.set(ControlMode.Position, angle);
  }
  public void rotateIntake(double _angle){
    double angle = _angle * k.ARM.intakeCntsPDeg;
    m_intakeRotateMotCtrl.set(ControlMode.Position, angle);
  }
  public void spinIntake(double _velocity){
    m_intakeSpinnerMotCtrl.set(ControlMode.PercentOutput, _velocity);
  }
  public void setShoulderAngle(double _angle){
    m_shoulderAngle = _angle;
  }
  public void setElbowAngle(double _angle){
    m_elbowAngle = _angle;
  }
  public void setIntakeAngle(double _angle){
    m_intakeAngle = _angle;
  }
  public void setIntakeVelocity(double _velocity){
    m_intakeVelocity = _velocity;
  }
  /** Return true if all motors are within target range */
  public boolean onTarget(){
    double shoulderAngle = m_leftShoulderMotCtrl.getSelectedSensorPosition();
    double elbowAngle = m_leftElbowMotCtrl.getSelectedSensorPosition();
    double intakeAngle = m_intakeRotateMotCtrl.getSelectedSensorPosition();

    // TODO: Check all angles within the range of where they need to be.
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
