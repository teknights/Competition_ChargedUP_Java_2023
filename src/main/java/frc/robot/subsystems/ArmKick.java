// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* This subsystem will be pushing(kicking) foward the arm so the arm is able to pick up items from the ground This should only
be fired once 
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmKick extends SubsystemBase {
  /** Creates a new ArmKick. */
  public static DoubleSolenoid armkicksolenoid = null;
  
  public ArmKick() 
  {
    armkicksolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Arm_Kick_Deploy, Constants.Arm_Kick_Retract);
  }
  

  
    public void DeployArmKick()
    {
      //deploys armkicksolenoid
    armkicksolenoid.set(Value.kForward);
    }
    public void RetractArmKick()
    {
      //retracts armkicksolenoid
    armkicksolenoid.set(Value.kReverse);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
