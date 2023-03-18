// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmDrive;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class HoldAtAngle extends CommandBase {
  double Angle;
  boolean isFinished = false;
  boolean inErrorZone = false;
  int count;
  /** Creates a new HoldAtAngle. */
  public HoldAtAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.m_ArmDrive);
    Angle = angle;
  }

  private void requires(ArmDrive m_ArmDrive) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_ArmDrive.rotateDegrees(Angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double error = Robot.m_ArmDrive.m_pid.gettolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
