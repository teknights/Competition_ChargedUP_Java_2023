package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{
    DoubleSolenoid Left_Claw_Solenoid= null;
    DoubleSolenoid Right_Claw_Solenoid = null;
    
    public Claw() 
  {
    /* This creates a new doubleSolenoid for the left and right sides of the claw sets them to the only pneumatics controller
    It also takes the input of the ports on the pneumatics module and grabs the int value from constants
    */
    Left_Claw_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Left_Claw_Open, Constants.Left_Claw_Close);
    Right_Claw_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Right_Claw_Open, Constants.Right_Claw_Close);
  }
    
    public void Left_Claw_Open()
    {
        //This sets the left claw to move in reverse which results in the claw opening
        Left_Claw_Solenoid.set(Value.kReverse);
    }
    public void Left_Claw_Close()
    {
        // This sets the left claw to move forward which results in the claw closing
        Left_Claw_Solenoid.set(Value.kForward);
    }
    public void Right_Claw_Open()
    {
        // This sets the right claw to move in reverse which results in the claw opening
        Right_Claw_Solenoid.set(Value.kReverse);   
    }
    public void Right_Claw_Close()
    {
        //This sets the right claw to move forward which results in the claw closing
        Right_Claw_Solenoid.set(Value.kForward);   
    }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
