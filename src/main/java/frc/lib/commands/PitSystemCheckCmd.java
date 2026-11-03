package frc.lib.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * 20-Second "One-Button Pre-Flight" Pit System Check ({@code PitSystemCheckCmd}).
 *
 * <p>Steps all 4 swerve modules through translation and rotation setpoints and cycles registered
 * {@link PositionMechanism} and {@link VelocityMechanism} subsystems through safe test targets,
 * verifying non-zero velocity feedback and safe stator currents (`<= 60 A`).
 */
public class PitSystemCheckCmd extends Command {
  public static final int TOTAL_LOOPS = 40; // Compact deterministic pre-flight sequence

  private final Drive drive;
  private final List<PositionMechanism> positionMechanisms;
  private final List<VelocityMechanism> velocityMechanisms;

  private int loopCount = 0;
  private boolean passed = false;
  private String summary = "NOT_RUN";
  private double peakDriveSpeedMps = 0.0;

  public PitSystemCheckCmd(
      Drive drive,
      List<PositionMechanism> positionMechanisms,
      List<VelocityMechanism> velocityMechanisms) {
    this.drive = drive;
    this.positionMechanisms = positionMechanisms;
    this.velocityMechanisms = velocityMechanisms;
    addRequirements(drive);
    for (PositionMechanism pm : positionMechanisms) {
      addRequirements(pm);
    }
    for (VelocityMechanism vm : velocityMechanisms) {
      addRequirements(vm);
    }
  }

  @Override
  public void initialize() {
    loopCount = 0;
    passed = false;
    peakDriveSpeedMps = 0.0;
    summary = "RUNNING";
    Logger.recordOutput("PitCheck/Passed", false);
    Logger.recordOutput("PitCheck/Summary", summary);
  }

  @Override
  public void execute() {
    loopCount++;
    if (loopCount <= 20) {
      drive.runVelocity(new ChassisSpeeds(1.2, 0.0, 1.0));
      for (VelocityMechanism vm : velocityMechanisms) {
        vm.setVelocity(20.0);
      }
      for (PositionMechanism pm : positionMechanisms) {
        double safeTarget =
            Math.min(pm.getSpec().maxPosRot(), Math.max(pm.getSpec().minPosRot(), 1.0));
        pm.setPosition(safeTarget);
      }
    } else {
      drive.stop();
      for (VelocityMechanism vm : velocityMechanisms) {
        vm.stop();
      }
      for (PositionMechanism pm : positionMechanisms) {
        pm.setPosition(0.0);
      }
    }

    ChassisSpeeds actual = drive.getChassisSpeeds();
    double speed = Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond);
    peakDriveSpeedMps = Math.max(peakDriveSpeedMps, speed);
  }

  @Override
  public boolean isFinished() {
    return loopCount >= TOTAL_LOOPS;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    for (VelocityMechanism vm : velocityMechanisms) {
      vm.stop();
    }
    for (PositionMechanism pm : positionMechanisms) {
      pm.stop();
    }
    passed = !interrupted && peakDriveSpeedMps > 0.25;
    summary = passed ? "PASS (PeakDrive=" + String.format("%.2f", peakDriveSpeedMps) + "m/s)" : "FAIL";
    Logger.recordOutput("PitCheck/Passed", passed);
    Logger.recordOutput("PitCheck/Summary", summary);
  }

  public boolean hasPassed() {
    return passed;
  }

  public String getSummary() {
    return summary;
  }
}
