// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoIntakeInCommand extends CommandBase {
  double currentLim = 20;
  double intakeSpeed = 4;
  Timer timer = new Timer();
  /** Creates a new AutoIntakeInCommand. */
  public AutoIntakeInCommand() {
    addRequirements(RobotContainer.intake);// here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.spinHand(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.intake.getIntakeCurrent() > 30 || timer.hasElapsed(8)){
      RobotContainer.intake.spinHand(0);
      return true;
    }
    return false;
  }
}
