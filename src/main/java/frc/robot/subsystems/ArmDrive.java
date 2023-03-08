// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmDrive extends SubsystemBase {
  // Creates a new ArmDrive. 
   static CANSparkMax ArmDriveMotor = new CANSparkMax(6, MotorType.kBrushed);   

public final void Arm_Drive()
{
  XboxController armController = new XboxController(1);
  double ArmMotorSpeed = armController.getRightY();
    ArmDriveMotor.set(armController.getRightY());
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
