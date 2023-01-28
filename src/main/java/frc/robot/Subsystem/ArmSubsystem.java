// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;

public class ArmSubsystem extends SubsystemBase {
  public CANSparkMax m_leftCanSparkMax;
  public CANSparkMax m_rightCanSparkMax;

  /**
   * The CANSparkMax has the ability to run it own PID control. Set the PID values and tell it to go to a position
   */
  /** Creates a new Arm. */
  public ArmSubsystem() {
    // m_leftCanSparkMax = new CANSparkMax(k.ARM.leftCANId, MotorType.kBrushless);
    // m_rightCanSparkMax = new CANSparkMax(k.ARM.rightCANId, MotorType.kBrushless);
    // m_rightCanSparkMax.follow(m_leftCanSparkMax, true);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
