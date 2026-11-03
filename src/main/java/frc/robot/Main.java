package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Entry point for Flux-SystemCore-2027 / Flux-RoboRIO-2027. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
