// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public WPI_TalonFX m_leftShoulderMotCtrl;
  public WPI_TalonFX m_rightShoulderMotCtrl;
  public WPI_TalonSRX m_leftElbowMotCtrl;
  public WPI_TalonSRX m_rightElbowMotCtrl;
  public WPI_TalonSRX m_intakeRotateMotCtrl;
  public WPI_TalonSRX m_intakeSpinnerMotCtrl;

  


  /** Creates a new Arm. */
  public Arm() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
