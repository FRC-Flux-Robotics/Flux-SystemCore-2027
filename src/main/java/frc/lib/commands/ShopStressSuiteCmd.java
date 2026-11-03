package frc.lib.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.lib.drive.VirtualGeofence;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.FieldProfile;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Automated 4-Capsule High-Intensity Shop Floor Torture Suite ({@code ShopStressSuiteCmd}) executed
 * inside the {@code 5m x 3m} shop micro-arena ({@link FieldProfile#SHOP_5X3}) under {@link
 * VirtualGeofence} protection:
 *
 * <ol>
 *   <li>{@code SUICIDE_SHUTTLE}: Hard X-axis launch &amp; snap reversal inside {@code +/- 1.35m}.
 *   <li>{@code BEYBLADE_FIGURE_8}: Simultaneous translation + {@code 540 deg/s} spin.
 *   <li>{@code VIRTUAL_TARGET_TRACK}: Perimeter strafe while tracking a virtual target.
 *   <li>{@code BOX_STEP_90_DEG}: Sharp 90-degree cornering around a {@code 2.0m x 0.8m} box.
 * </ol>
 */
public class ShopStressSuiteCmd extends Command {
  public static final int LOOPS_PER_CAPSULE = 20;
  public static final int TOTAL_LOOPS = LOOPS_PER_CAPSULE * 4;

  public enum Capsule {
    SUICIDE_SHUTTLE,
    BEYBLADE_FIGURE_8,
    VIRTUAL_TARGET_TRACK,
    BOX_STEP_90_DEG
  }

  private final Drive drive;
  private final BooleanSupplier deadManHeldSupplier;

  private FieldProfile previousProfile = FieldProfile.FULL_FIELD;
  private int loopCount = 0;
  private int wallBreachCount = 0;
  private Capsule activeCapsule = Capsule.SUICIDE_SHUTTLE;

  public ShopStressSuiteCmd(Drive drive, BooleanSupplier deadManHeldSupplier) {
    this.drive = drive;
    this.deadManHeldSupplier = deadManHeldSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    loopCount = 0;
    wallBreachCount = 0;
    previousProfile = FieldPositions.getFieldProfile();
    FieldPositions.setFieldProfile(FieldProfile.SHOP_5X3);
    Logger.recordOutput("ShopStressSuite/Completed", false);
    Logger.recordOutput("ShopStressSuite/WallBreaches", 0);
  }

  @Override
  public void execute() {
    if (!deadManHeldSupplier.getAsBoolean()) {
      drive.stopWithX();
      return;
    }

    loopCount++;
    int capsuleIdx = Math.min(3, (loopCount - 1) / LOOPS_PER_CAPSULE);
    activeCapsule = Capsule.values()[capsuleIdx];

    ChassisSpeeds desiredFieldSpeeds =
        switch (activeCapsule) {
          case SUICIDE_SHUTTLE ->
              new ChassisSpeeds((loopCount % 10 < 5) ? 3.2 : -3.2, 0.0, 0.0);
          case BEYBLADE_FIGURE_8 ->
              new ChassisSpeeds(
                  1.5 * Math.cos(loopCount * 0.3),
                  0.6 * Math.sin(loopCount * 0.6),
                  Math.toRadians(540.0));
          case VIRTUAL_TARGET_TRACK ->
              new ChassisSpeeds(0.0, (loopCount % 10 < 5) ? 1.8 : -1.8, 2.2);
          case BOX_STEP_90_DEG -> {
            int leg = loopCount % 4;
            if (leg == 0) yield new ChassisSpeeds(2.2, 0.0, 0.0);
            if (leg == 1) yield new ChassisSpeeds(0.0, 1.4, 0.0);
            if (leg == 2) yield new ChassisSpeeds(-2.2, 0.0, 0.0);
            yield new ChassisSpeeds(0.0, -1.4, 0.0);
          }
        };

    drive.runFieldRelative(desiredFieldSpeeds, true, drive.getLastVisionSightAgeSec());

    Pose2d p = drive.getPose();
    if (Math.abs(p.getX()) > VirtualGeofence.SAFE_HALF_X_METERS + 0.05
        || Math.abs(p.getY()) > VirtualGeofence.SAFE_HALF_Y_METERS + 0.05) {
      wallBreachCount++;
    }

    Logger.recordOutput("ShopStressSuite/ActiveCapsule", activeCapsule.name());
    Logger.recordOutput("ShopStressSuite/WallBreaches", wallBreachCount);
  }

  @Override
  public boolean isFinished() {
    return loopCount >= TOTAL_LOOPS;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    FieldPositions.setFieldProfile(previousProfile);
    Logger.recordOutput("ShopStressSuite/Completed", !interrupted && loopCount >= TOTAL_LOOPS);
  }

  public int getWallBreachCount() {
    return wallBreachCount;
  }

  public Capsule getActiveCapsule() {
    return activeCapsule;
  }
}
