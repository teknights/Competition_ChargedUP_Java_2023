// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class ArmDrive extends SubsystemBase {
  // Creates a new ArmDrive. 
   public static CANSparkMax ArmDriveMotor = new CANSparkMax(6, MotorType.kBrushed);   

public void Arm_Drive()
{
  double ArmSpeed;
  XboxController armController = new XboxController(1);
  double armrightydeadband = MathUtil.applyDeadband(armController.getRightY(), 0.3);
  ArmSpeed = armrightydeadband;
    ArmDriveMotor.set(-armrightydeadband);
    System.out.println(-armrightydeadband);
    //this is used for diagnostic purposes

    if (ArmSpeed != 0) {
      if (RobotContainer.Maxheight.get()) {
        if (ArmSpeed > 0){
          ArmDriveMotor.set(0);

        }

      }
    }
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
