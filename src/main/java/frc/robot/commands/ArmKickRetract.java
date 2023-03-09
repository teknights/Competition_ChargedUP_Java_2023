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
public class ArmKickRetract extends InstantCommand {
  public ArmKickRetract() {
    super();
    /* super eliminates the confusion between superclasses and subclasses 
    that have methods with the same or similar names. */
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //this requires the armkick subsystem
    requires(Robot.m_ArmKick);
}

private void requires(ArmKick m_ArmKick) {
}  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //this retracts the ArmKick Solenoid
    Robot.m_ArmKick.RetractArmKick();

  }
}
