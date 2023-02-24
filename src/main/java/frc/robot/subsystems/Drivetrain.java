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
    {
        XboxController drivController = new XboxController(0);
        drivetrain.arcadeDrive(-drivController.getRightX(), -drivController.getLeftY());
    }
}
//robot is driving