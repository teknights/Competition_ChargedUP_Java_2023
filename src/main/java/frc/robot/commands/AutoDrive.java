// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
  double Target;
  /** Creates a new AutoDrive. */
  public AutoDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Target = 0;
    if(RobotContainer.IMU.getAngle() !=Target){
      while(RobotContainer.IMU.getAngle() !=Target){
        Drivetrain.arcadeDrive(0.1, -0.1);
      }
    }
    Drivetrain.arcadeDrive(-0.1, -0.1);
    Timer.delay(5);
    Drivetrain.arcadeDrive(0, 0);
    Target = 90;
    if(RobotContainer.IMU.getAngle() !=Target){
      while(RobotContainer.IMU.getAngle() !=Target){
        Drivetrain.arcadeDrive(0.1, -0.1);
      }
    }
    Drivetrain.arcadeDrive(-0.1, -0.1);
    Timer.delay(5);
    Drivetrain.arcadeDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
    
  }
}
