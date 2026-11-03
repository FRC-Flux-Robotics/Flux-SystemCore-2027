package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

/**
 * Abstracts WPILib 2026 Wall/Blue vs. WPILib 2027 Center/Red coordinate origins and Full-Field,
 * Half-Field, and 5m x 3m Shop Micro-Arena bounds.
 */
public final class FieldPositions {
  public static final double FIELD_LENGTH_METERS = 16.541;
  public static final double FIELD_WIDTH_METERS = 8.211;
  public static final double HALF_FIELD_LENGTH_METERS = FIELD_LENGTH_METERS * 0.5;
  public static final double HALF_FIELD_WIDTH_METERS = FIELD_WIDTH_METERS * 0.5;
  public static final double SHOP_LENGTH_METERS = 5.000;
  public static final double SHOP_WIDTH_METERS = 3.000;

  private static double shopMinX = -SHOP_LENGTH_METERS * 0.5;
  private static double shopMaxX = SHOP_LENGTH_METERS * 0.5;
  private static double shopMinY = -SHOP_WIDTH_METERS * 0.5;
  private static double shopMaxY = SHOP_WIDTH_METERS * 0.5;

  public enum CoordinateSystem {
    WALL_BLUE_2026,
    CENTER_RED_2027
  }

  public enum FieldProfile {
    FULL_FIELD,
    HALF_FIELD_BLUE,
    HALF_FIELD_RED,
    SHOP_5X3
  }

  private static CoordinateSystem coordinateSystem = CoordinateSystem.CENTER_RED_2027;
  private static FieldProfile fieldProfile = FieldProfile.FULL_FIELD;
  private static Boolean redAllianceOverride = null;

  private FieldPositions() {}

  public static CoordinateSystem getCoordinateSystem() {
    return coordinateSystem;
  }

  public static void setCoordinateSystem(CoordinateSystem system) {
    coordinateSystem = (system != null) ? system : CoordinateSystem.CENTER_RED_2027;
    Logger.recordOutput("Field/CoordinateSystem", coordinateSystem.name());
  }

  /**
   * Returns the active field profile. Automatically locks to {@link FieldProfile#FULL_FIELD} when
   * connected to FMS at competition.
   */
  public static FieldProfile getFieldProfile() {
    if (DriverStation.isFMSAttached()) {
      return FieldProfile.FULL_FIELD;
    }
    return fieldProfile;
  }

  /** Sets symmetric shop/arena dimensions centered on `(0, 0)` (e.g., from `parameters.csv`). */
  public static void setShopDimensions(double lengthMeters, double widthMeters) {
    if (lengthMeters <= 1.0 || widthMeters <= 1.0) {
      throw new IllegalArgumentException(
          "Shop dimensions must be > 1.0m (got " + lengthMeters + "m x " + widthMeters + "m)");
    }
    shopMinX = -lengthMeters / 2.0;
    shopMaxX = lengthMeters / 2.0;
    shopMinY = -widthMeters / 2.0;
    shopMaxY = widthMeters / 2.0;
  }

  /**
   * Sets asymmetric auto-measured arena wall bounds `[minX, maxX] x [minY, maxY]` in meters (e.g.,
   * measured from wall AprilTags during `ZeroPriorBootstrapCmd`).
   */
  public static void setCustomArenaBounds(double minX, double maxX, double minY, double maxY) {
    if (maxX <= minX || maxY <= minY) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid arena bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]", minX, maxX, minY, maxY));
    }
    shopMinX = minX;
    shopMaxX = maxX;
    shopMinY = minY;
    shopMaxY = maxY;
  }

  /** Resets shop/arena bounds to the default 5.0m x 3.0m (`[-2.5, +2.5] x [-1.5, +1.5]`). */
  public static void resetShopDimensions() {
    shopMinX = -SHOP_LENGTH_METERS / 2.0;
    shopMaxX = SHOP_LENGTH_METERS / 2.0;
    shopMinY = -SHOP_WIDTH_METERS / 2.0;
    shopMaxY = SHOP_WIDTH_METERS / 2.0;
  }

  public static double getShopMinX() {
    return shopMinX;
  }

  public static double getShopMaxX() {
    return shopMaxX;
  }

  public static double getShopMinY() {
    return shopMinY;
  }

  public static double getShopMaxY() {
    return shopMaxY;
  }

  public static double getShopLengthMeters() {
    return shopMaxX - shopMinX;
  }

  public static double getShopWidthMeters() {
    return shopMaxY - shopMinY;
  }

  public static void setFieldProfile(FieldProfile profile) {
    fieldProfile = (profile != null) ? profile : FieldProfile.FULL_FIELD;
    Logger.recordOutput("Field/Profile", fieldProfile.name());
  }

  public static void setAllianceOverride(Boolean isRed) {
    redAllianceOverride = isRed;
  }

  /** Converts a Pose2d from 2026 Wall/Blue origin to 2027 Center/Red origin. */
  public static Pose2d wallBlueToCenterRed(Pose2d wallBlue) {
    double cx = FIELD_LENGTH_METERS * 0.5 - wallBlue.getX();
    double cy = FIELD_WIDTH_METERS * 0.5 - wallBlue.getY();
    Rotation2d rot = wallBlue.getRotation().plus(Rotation2d.kPi);
    return new Pose2d(new Translation2d(cx, cy), rot);
  }

  /** Converts a Pose2d from 2027 Center/Red origin back to 2026 Wall/Blue origin. */
  public static Pose2d centerRedToWallBlue(Pose2d centerRed) {
    double wx = FIELD_LENGTH_METERS * 0.5 - centerRed.getX();
    double wy = FIELD_WIDTH_METERS * 0.5 - centerRed.getY();
    Rotation2d rot = centerRed.getRotation().plus(Rotation2d.kPi);
    return new Pose2d(new Translation2d(wx, wy), rot);
  }

  /** Converts a canonical 2026 Wall/Blue pose into the currently active {@link CoordinateSystem}. */
  public static Pose2d fromWallBlue(Pose2d wallBluePose) {
    return (coordinateSystem == CoordinateSystem.CENTER_RED_2027)
        ? wallBlueToCenterRed(wallBluePose)
        : wallBluePose;
  }

  /** Converts a canonical 2026 Wall/Blue translation into the active {@link CoordinateSystem}. */
  public static Translation2d fromWallBlue(Translation2d wallBlueTranslation) {
    if (coordinateSystem == CoordinateSystem.WALL_BLUE_2026) {
      return wallBlueTranslation;
    }
    return new Translation2d(
        HALF_FIELD_LENGTH_METERS - wallBlueTranslation.getX(),
        HALF_FIELD_WIDTH_METERS - wallBlueTranslation.getY());
  }

  /** Converts a pose from the active {@link CoordinateSystem} back to 2026 Wall/Blue. */
  public static Pose2d toWallBlue(Pose2d activePose) {
    return (coordinateSystem == CoordinateSystem.CENTER_RED_2027)
        ? centerRedToWallBlue(activePose)
        : activePose;
  }

  public static boolean isInsideField(double x, double y, double marginMeters) {
    return isInsideField(x, y, marginMeters, coordinateSystem, getFieldProfile());
  }

  public static boolean isInsideField(
      double x, double y, double marginMeters, CoordinateSystem sys, FieldProfile profile) {
    if (profile == FieldProfile.SHOP_5X3) {
      return x >= (shopMinX - marginMeters)
          && x <= (shopMaxX + marginMeters)
          && y >= (shopMinY - marginMeters)
          && y <= (shopMaxY + marginMeters);
    }
    double blueX = (sys == CoordinateSystem.CENTER_RED_2027) ? (HALF_FIELD_LENGTH_METERS - x) : x;
    double blueY = (sys == CoordinateSystem.CENTER_RED_2027) ? (HALF_FIELD_WIDTH_METERS - y) : y;

    double minX = (profile == FieldProfile.HALF_FIELD_RED) ? HALF_FIELD_LENGTH_METERS : 0.0;
    double maxX =
        (profile == FieldProfile.HALF_FIELD_BLUE) ? HALF_FIELD_LENGTH_METERS : FIELD_LENGTH_METERS;
    return blueX >= (minX - marginMeters)
        && blueX <= (maxX + marginMeters)
        && blueY >= -marginMeters
        && blueY <= (FIELD_WIDTH_METERS + marginMeters);
  }

  public static boolean isRedAlliance() {
    FieldProfile profile = getFieldProfile();
    if (profile == FieldProfile.HALF_FIELD_BLUE) {
      return false;
    }
    if (profile == FieldProfile.HALF_FIELD_RED) {
      return true;
    }
    if (redAllianceOverride != null && !DriverStation.isFMSAttached()) {
      return redAllianceOverride;
    }
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /** Mirrors a pose across midfield in the active {@link CoordinateSystem}. */
  public static Pose2d mirror(Pose2d pose) {
    if (coordinateSystem == CoordinateSystem.CENTER_RED_2027) {
      return new Pose2d(-pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.kPi));
    }
    return new Pose2d(
        FIELD_LENGTH_METERS - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.kPi));
  }

  /** Mirrors a pose rotationally (both X and Y) around field center in the active frame. */
  public static Pose2d mirrorRotational(Pose2d pose) {
    if (coordinateSystem == CoordinateSystem.CENTER_RED_2027) {
      return new Pose2d(-pose.getX(), -pose.getY(), pose.getRotation().plus(Rotation2d.kPi));
    }
    return new Pose2d(
        FIELD_LENGTH_METERS - pose.getX(),
        FIELD_WIDTH_METERS - pose.getY(),
        pose.getRotation().plus(Rotation2d.kPi));
  }

  /** Resolves a canonical Blue-alliance pose (in Wall/Blue) for the active alliance and frame. */
  public static Pose2d forAlliance(Pose2d blueWallPose) {
    Pose2d active = fromWallBlue(blueWallPose);
    return isRedAlliance() ? mirror(active) : active;
  }

  /** Resolves a canonical Blue-alliance translation (in Wall/Blue) for the active alliance. */
  public static Translation2d forAlliance(Translation2d blueWallTranslation) {
    Translation2d active = fromWallBlue(blueWallTranslation);
    if (!isRedAlliance()) {
      return active;
    }
    if (coordinateSystem == CoordinateSystem.CENTER_RED_2027) {
      return new Translation2d(-active.getX(), active.getY());
    }
    return new Translation2d(FIELD_LENGTH_METERS - active.getX(), active.getY());
  }

  /** Returns the operator forward heading for the current alliance in the active frame. */
  public static Rotation2d operatorForward() {
    boolean red = isRedAlliance();
    if (coordinateSystem == CoordinateSystem.CENTER_RED_2027) {
      return red ? Rotation2d.kZero : Rotation2d.kPi;
    }
    return red ? Rotation2d.kPi : Rotation2d.kZero;
  }

  /** Configures an {@link AprilTagFieldLayout} origin to match the active coordinate system. */
  public static AprilTagFieldLayout configureLayoutOrigin(AprilTagFieldLayout layout) {
    if (coordinateSystem == CoordinateSystem.CENTER_RED_2027) {
      layout.setOrigin(
          new Pose3d(
              HALF_FIELD_LENGTH_METERS,
              HALF_FIELD_WIDTH_METERS,
              0.0,
              new Rotation3d(0.0, 0.0, Math.PI)));
    } else {
      layout.setOrigin(new Pose3d());
    }
    return layout;
  }
}
