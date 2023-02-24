package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.*;

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robotInit() method in your program.
 */
public class CameraCode extends TimedRobot {
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
  }
}
