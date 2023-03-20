// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }
  // Solenoids for ArmKick
public static final int Arm_Kick_Deploy = 0;
public static final int Arm_Kick_Retract = 1;
//Solenoids for Claw
public static final int Left_Claw_Open = 2;
public static final int Left_Claw_Close = 3;
public static final int Right_Claw_Open = 4;
public static final int Right_Claw_Close = 5;
//Can IDs
  public static final int Left_Front_Motor_ID = 3;
  public static final int Left_Back_Motor_ID = 2;
  public static final int Right_Front_Motor_ID = 5;
  public static final int Right_Back_Motor_ID = 4;
  public static final int Arm_Motor_ID = 6;
  public static int kArmControllerPort;
  //Limit Switches DIO Channels
  public static final int VerticalLimitFoldedIn = 2;
  public static final int VerticalLimitFoldedOut = 3;
  public static final int NoTowerLean = 0;
  public static final int FloorHit = 1;
}

