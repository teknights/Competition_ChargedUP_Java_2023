// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;


import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Left_Claw_Open extends InstantCommand {
  public Left_Claw_Open() {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.m_Claw);
    
  }

  private void requires(Claw m_Claw) {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    Robot.m_Claw.Left_Claw_Open();
  }
}
