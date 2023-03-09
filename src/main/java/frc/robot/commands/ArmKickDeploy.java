// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ArmKick;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmKickDeploy extends InstantCommand {
  public ArmKickDeploy() {
    super();
       // Use requires() here to declare subsystem dependencies
       // eg. requires(chassis);
       //this command requires the ArmKick subsystem to execute the command
      requires(Robot.m_ArmKick);
  }

  private void requires(ArmKick m_ArmKick) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_ArmKick.DeployArmKick();
  }
}
