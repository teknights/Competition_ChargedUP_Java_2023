// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.SocketTimeoutException;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmDrive;
import frc.robot.subsystems.*;

public class AutoArmDrive extends CommandBase {
     
  /** Creates a new AutoArmDrive. */
  public AutoArmDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     double RunTime;
     //RunTime is the number of seconds that it runs for
     RunTime = 1.5;
     //Arm motor is set to 15% power for 1.5 seconds then sets motor power to zero
    ArmDrive.ArmDriveMotor.set(0.15);
    Timer.delay(RunTime);
    ArmDrive.ArmDriveMotor.set(0);
  }
      

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
