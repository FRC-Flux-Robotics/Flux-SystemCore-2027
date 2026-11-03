package frc.robot.mechanisms;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * 2027-specific 2-stage cascading elevator + arm forward kinematics, 2D crossbar keep-out
 * interlocks, and dynamic CG anti-tipping swerve acceleration scaling.
 */
public final class ElevatorArmKinematics {
  public static final double ELEVATOR_METERS_PER_ROTATION = 0.030;
  public static final double BASE_PIVOT_Z_METERS = 0.25;
  public static final double ARM_LENGTH_METERS = 0.48;
  public static final double CROSSBAR_HEIGHT_METERS = 0.42;
  public static final double CROSSBAR_CLEARANCE_MARGIN_METERS = 0.03;
  public static final double MIN_SAFE_ELEVATOR_ROTATIONS =
      (CROSSBAR_HEIGHT_METERS + CROSSBAR_CLEARANCE_MARGIN_METERS) / ELEVATOR_METERS_PER_ROTATION;
  public static final double SAFE_STOW_MIN_RAD = -0.15;
  public static final double SAFE_STOW_MAX_RAD = 0.15;
  public static final double MAX_CARRIAGE_HEIGHT_METERS = 1.40;
  public static final double TIP_SCALER_K = 0.65;

  private ElevatorArmKinematics() {}

  /** Computes Stage 1 height for a 2-stage cascading rigging (Stage 1 moves at 0.5x carriage). */
  public static double stage1HeightMeters(double carriageHeightMeters) {
    return 0.5 * Math.max(0.0, carriageHeightMeters);
  }

  /** Computes end-effector (x_tip, z_tip) in robot frame from carriage height and arm angle. */
  public static Translation2d forwardKinematics(double carriageHeightMeters, double armAngleRad) {
    double xTip = ARM_LENGTH_METERS * Math.cos(armAngleRad);
    double zTip =
        BASE_PIVOT_Z_METERS + carriageHeightMeters + ARM_LENGTH_METERS * Math.sin(armAngleRad);
    return new Translation2d(xTip, zTip);
  }

  /** Clamps desired arm angle to prevent colliding with the elevator crossbar when low. */
  public static double clampSafeArmAngleRad(double carriageHeightMeters, double desiredArmRad) {
    if (carriageHeightMeters < CROSSBAR_HEIGHT_METERS) {
      return Math.max(SAFE_STOW_MIN_RAD, Math.min(SAFE_STOW_MAX_RAD, desiredArmRad));
    }
    return desiredArmRad;
  }

  /** Scales maximum swerve linear acceleration as a function of elevator carriage height. */
  public static double maxSafeAccelerationMps2(double baseAccelMps2, double carriageHeightMeters) {
    double normH = Math.max(0.0, Math.min(1.0, carriageHeightMeters / MAX_CARRIAGE_HEIGHT_METERS));
    return baseAccelMps2 * (1.0 - TIP_SCALER_K * normH);
  }

  /** Returns normalized [1 - TIP_SCALER_K, 1.0] acceleration scale factor for a carriage height. */
  public static double maxAccelScaleFactor(double carriageHeightMeters) {
    return maxSafeAccelerationMps2(1.0, carriageHeightMeters);
  }

  /** Returns true if the arm angle (degrees) does not violate the crossbar keep-out zone. */
  public static boolean isArmSafe(double carriageHeightMeters, double desiredArmDeg) {
    double rad = Math.toRadians(desiredArmDeg);
    return Math.abs(clampSafeArmAngleRad(carriageHeightMeters, rad) - rad) <= 1e-6;
  }
}
