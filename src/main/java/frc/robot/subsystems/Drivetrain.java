package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
    CANSparkMax left_front = new CANSparkMax(3, MotorType.kBrushed);
    CANSparkMax left_back = new CANSparkMax(2, MotorType.kBrushed);
    CANSparkMax right_front = new CANSparkMax(5, MotorType.kBrushed);
    CANSparkMax right_back = new CANSparkMax(4, MotorType.kBrushed);

    DifferentialDrive drivetrain = new DifferentialDrive
    (
        new MotorControllerGroup(left_front, left_back),
        new MotorControllerGroup(right_front, right_back)
    );

    public void drive(double xSpeed, double zRotation)
    {//this creates the driv xbox controller in port 0
        XboxController drivController = new XboxController(0);
        /*This takes the input from the controller and sets the motor to that value. You may
        notice that it is being multiplied by itself. This is on purpose as it makes the
        input exponetial to essentially create a ramp on the motots as it will be less sensitve
        near the center of the drive stick and more sensitve the farther you push the joysticks
        */
        drivetrain.arcadeDrive(-drivController.getRightX()*drivController.getRightX(), -drivController.getLeftY()*drivController.getLeftY());
    }
}
//robot is driving