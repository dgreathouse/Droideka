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
  public WPI_TalonFX m_leftBicepMotCtrl;
  public WPI_TalonFX m_rightBicepMotCtrl;
  public WPI_TalonSRX m_leftElbowMotCtrl;
  public WPI_TalonSRX m_rightElbowMotCtrl;
  public WPI_TalonSRX m_intakeRotateMotCtrl;
  public WPI_TalonSRX m_intakeSpinnerMotCtrl;

  public ArmPosEnum m_armPos = ArmPosEnum.HOME;
  public double m_elbowAngle = 0;
  public double m_bicepAngle = 0;
  public double m_intakeAngle = 0;
  public double m_intakeVelocity = 0;
  /**
   * 
   */
  /** Creates a new Arm. */
  public ArmSubsystem() {
    /************************ BICEP ****************************/
    // Create new instance of motor control classes 
    m_leftBicepMotCtrl = new WPI_TalonFX(k.ARM.leftBicepCANId);
    m_rightBicepMotCtrl = new WPI_TalonFX(k.ARM.rightBicepCANId);

    // Invert the other controller that is going to follow
    m_rightBicepMotCtrl.setInverted(InvertType.InvertMotorOutput);
    m_rightBicepMotCtrl.follow(m_leftBicepMotCtrl);

    // Configure the closedloop gain values
    m_leftBicepMotCtrl.config_kP(0, 0.5);
    m_leftBicepMotCtrl.config_kI(0, 0.0);
    m_leftBicepMotCtrl.config_kD(0, 0.0);
    m_leftBicepMotCtrl.configClosedLoopPeakOutput(0, 0.75);
    m_leftBicepMotCtrl.configClosedloopRamp(1);

    /************************ ELBOW ****************************/
    // Create new instance of motor control classes 
    m_leftElbowMotCtrl = new WPI_TalonSRX(k.ARM.leftElbowCANId);
    m_rightElbowMotCtrl = new WPI_TalonSRX(k.ARM.rightElbowCANId);

    // Invert the other controller that is going to follow
    m_rightElbowMotCtrl.setInverted(InvertType.InvertMotorOutput);
    m_rightElbowMotCtrl.follow(m_leftBicepMotCtrl);

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

    m_armPos = ArmPosEnum.HOME;
  }
  public void rotateBicep(double _angle){
    double angle = _angle * k.ARM.bicepCntsPDeg;
    m_leftBicepMotCtrl.set(ControlMode.Position, angle);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
