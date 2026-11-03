package frc.lib.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.CoordinateSystem;
import frc.lib.util.FieldPositions.FieldProfile;

/**
 * Kinematic braking barrier and Dead-Man watchdog for confined shop/arena (`FieldProfile.SHOP_5X3`)
 * and half-field practice (`HALF_FIELD_BLUE`, `HALF_FIELD_RED`).
 *
 * <p>Supports both the default `5m x 3m` room (safe robot center box `3.3m x 1.3m`) and dynamic
 * asymmetric auto-measured room bounds `[minX, maxX] x [minY, maxY]` from {@link FieldPositions},
 * subtracting the `0.85m` bumper + wall safety clearance from each wall.
 */
public final class VirtualGeofence {
  public static final double TOTAL_CLEARANCE_METERS = 0.85;
  public static final double SAFE_HALF_X_METERS = 1.65;
  public static final double SAFE_HALF_Y_METERS = 0.65;
  public static final double SAFE_BRAKE_DECEL_MPS2 = 4.5;
  public static final double LATENCY_LOOKAHEAD_SEC = 0.040;
  public static final double MAX_TAG_AGE_SEC = 0.500;

  private VirtualGeofence() {}

  /**
   * Clamps field-relative velocity commands so the robot can never cross the active shop/arena
   * virtual wall boundaries (`[ShopMinX + 0.85, ShopMaxX - 0.85] x [ShopMinY + 0.85, ShopMaxY -
   * 0.85]`, defaulting to `+/-1.65m X` and `+/-0.65m Y`).
   */
  public static ChassisSpeeds clampFieldSpeeds(
      double robotX,
      double robotY,
      ChassisSpeeds desiredFieldSpeeds,
      boolean deadManHeld,
      double lastTagSightAgeSec) {
    if (!deadManHeld || lastTagSightAgeSec > MAX_TAG_AGE_SEC) {
      return new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    double safeMinX = FieldPositions.getShopMinX() + TOTAL_CLEARANCE_METERS;
    double safeMaxX = FieldPositions.getShopMaxX() - TOTAL_CLEARANCE_METERS;
    double safeMinY = FieldPositions.getShopMinY() + TOTAL_CLEARANCE_METERS;
    double safeMaxY = FieldPositions.getShopMaxY() - TOTAL_CLEARANCE_METERS;

    double vx = clampAxisRange(robotX, safeMinX, safeMaxX, desiredFieldSpeeds.vxMetersPerSecond);
    double vy = clampAxisRange(robotY, safeMinY, safeMaxY, desiredFieldSpeeds.vyMetersPerSecond);
    return new ChassisSpeeds(vx, vy, desiredFieldSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Clamps field-relative speeds according to the active {@link FieldProfile}:
   *
   * <ul>
   *   <li>{@link FieldProfile#FULL_FIELD}: Geofence OFF (returns {@code desiredFieldSpeeds}).
   *   <li>{@link FieldProfile#SHOP_5X3}: Full 4-wall arena braking + dead-man + tag-loss guard.
   *   <li>{@link FieldProfile#HALF_FIELD_BLUE} / {@link FieldProfile#HALF_FIELD_RED}: Guards the
   *       open midfield centerline and perimeter walls so the robot does not cross midfield during
   *       half-field practice.
   * </ul>
   */
  public static ChassisSpeeds clampProfileSpeeds(
      double robotX,
      double robotY,
      ChassisSpeeds desiredFieldSpeeds,
      boolean deadManHeld,
      double lastTagSightAgeSec,
      CoordinateSystem coordSys,
      FieldProfile profile) {
    if (profile == FieldProfile.FULL_FIELD) {
      return desiredFieldSpeeds;
    }
    if (profile == FieldProfile.SHOP_5X3) {
      return clampFieldSpeeds(
          robotX, robotY, desiredFieldSpeeds, deadManHeld, lastTagSightAgeSec);
    }

    double minX;
    double maxX;
    double minY;
    double maxY;
    if (coordSys == CoordinateSystem.CENTER_RED_2027) {
      minY = -FieldPositions.HALF_FIELD_WIDTH_METERS + TOTAL_CLEARANCE_METERS;
      maxY = FieldPositions.HALF_FIELD_WIDTH_METERS - TOTAL_CLEARANCE_METERS;
      if (profile == FieldProfile.HALF_FIELD_BLUE) {
        minX = TOTAL_CLEARANCE_METERS;
        maxX = FieldPositions.HALF_FIELD_LENGTH_METERS - TOTAL_CLEARANCE_METERS;
      } else {
        minX = -FieldPositions.HALF_FIELD_LENGTH_METERS + TOTAL_CLEARANCE_METERS;
        maxX = -TOTAL_CLEARANCE_METERS;
      }
    } else {
      minY = TOTAL_CLEARANCE_METERS;
      maxY = FieldPositions.FIELD_WIDTH_METERS - TOTAL_CLEARANCE_METERS;
      if (profile == FieldProfile.HALF_FIELD_BLUE) {
        minX = TOTAL_CLEARANCE_METERS;
        maxX = FieldPositions.HALF_FIELD_LENGTH_METERS - TOTAL_CLEARANCE_METERS;
      } else {
        minX = FieldPositions.HALF_FIELD_LENGTH_METERS + TOTAL_CLEARANCE_METERS;
        maxX = FieldPositions.FIELD_LENGTH_METERS - TOTAL_CLEARANCE_METERS;
      }
    }

    double vx = clampAxisRange(robotX, minX, maxX, desiredFieldSpeeds.vxMetersPerSecond);
    double vy = clampAxisRange(robotY, minY, maxY, desiredFieldSpeeds.vyMetersPerSecond);
    return new ChassisSpeeds(vx, vy, desiredFieldSpeeds.omegaRadiansPerSecond);
  }

  private static double clampAxisRange(
      double pos, double minBound, double maxBound, double vDesired) {
    if (maxBound <= minBound) {
      return 0.0;
    }
    if (vDesired > 0.0) {
      double distToWall = maxBound - pos - vDesired * LATENCY_LOOKAHEAD_SEC;
      double vMax = Math.sqrt(2.0 * SAFE_BRAKE_DECEL_MPS2 * Math.max(0.0, distToWall));
      return Math.min(vDesired, vMax);
    } else if (vDesired < 0.0) {
      double distToWall = pos - minBound - (-vDesired) * LATENCY_LOOKAHEAD_SEC;
      double vMax = Math.sqrt(2.0 * SAFE_BRAKE_DECEL_MPS2 * Math.max(0.0, distToWall));
      return Math.max(vDesired, -vMax);
    }
    return 0.0;
  }
}
