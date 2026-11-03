package frc.simharness;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drive.Drive;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.FieldProfile;
import frc.lib.util.SafeAutoBuilder;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * JUnit test-scope autonomous simulation harness that steps {@link RobotContainer} through a
 * 15.0-second (750-loop at 20 ms) autonomous routine and exports {@code student-auto-frames.json}
 * for the standalone {@code FluxProcessor} video renderer ({@code java -jar flux-processor.jar
 * --render-auto}).
 *
 * <p>Kept strictly in {@code src/test/java/} so zero simulation harness or video-rendering code is
 * compiled into {@code frcUserProgram.jar} on the robot.
 */
public final class StudentAutoSimExporter {

  /** Single 25 Hz (40 ms) snapshot of the simulated robot during an autonomous run. */
  public record AutoFrame(
      double timeSec,
      double xMeters,
      double yMeters,
      double headingRad,
      double vxMps,
      double vyMps,
      double omegaRadPerSec,
      double elevatorHeightM,
      double armAngleRad,
      double intakeRollerRps,
      double batteryVoltage,
      double driveStatorAmps,
      double accelScale,
      String stateLabel) {}

  private StudentAutoSimExporter() {}

  /**
   * Steps {@code container} through {@code autoCommand} for 15.0 seconds (750 loops at 20 ms) and
   * writes {@code student-auto-frames.json} to {@code outputDir}.
   */
  public static List<AutoFrame> simulateAndExportJson(
      RobotContainer container, Command autoCommand, String routineName, Path outputDir)
      throws IOException {
    Drive drive = container.getDrive();
    Superstructure superstructure = container.getSuperstructure();

    FieldProfile previousProfile = FieldPositions.getFieldProfile();
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);
    SafeAutoBuilder.clearLastDiagnostic();
    try {
      drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      superstructure.requestState(State.STOWED);

      List<AutoFrame> frames = new ArrayList<>(380);
      autoCommand.initialize();

      int totalLoops = 750; // 15.0 seconds @ 50 Hz (20 ms)
      for (int step = 0; step <= totalLoops; step++) {
        double t = step * 0.020;
        if (!autoCommand.isFinished()) {
          autoCommand.execute();
        }
        drive.periodic();
        container.getElevator().periodic();
        container.getArmPivot().periodic();
        container.getIntakeRoller().periodic();
        superstructure.periodic();
        container.periodicUpdate();

        if (step % 2 == 0) {
          Pose2d p = drive.getPose();
          ChassisSpeeds cs =
              ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), p.getRotation());
          double elevMeters =
              container.getElevator().getPosition() * MechanismIOSim.ELEVATOR_METERS_PER_ROTATION;
          double armRad = container.getArmPivot().getPosition() * 2.0 * Math.PI;
          double rollerRps = container.getIntakeRoller().getVelocity();
          frames.add(
              new AutoFrame(
                  t,
                  p.getX(),
                  p.getY(),
                  p.getRotation().getRadians(),
                  cs.vxMetersPerSecond,
                  cs.vyMetersPerSecond,
                  cs.omegaRadiansPerSecond,
                  elevMeters,
                  armRad,
                  rollerRps,
                  container.getVirtualBatterySim().terminalVoltageV(),
                  drive.getTotalSupplyCurrentAmps()
                      + superstructure.getTotalMechanismSupplyCurrentAmps(),
                  superstructure.getSafeSwerveAccelerationScale(),
                  superstructure.getState().name()));
        }
      }
      autoCommand.end(false);

      String autoWarning = SafeAutoBuilder.getLastCancelReason();
      Files.createDirectories(outputDir);
      writeFramesJson(
          outputDir.resolve("student-auto-frames.json"),
          routineName,
          container.getIdentity().name(),
          frames,
          autoWarning);
      return frames;
    } finally {
      FieldPositions.setFieldProfile(previousProfile);
    }
  }

  private static void writeFramesJson(
      Path jsonFile,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      String autoWarning)
      throws IOException {
    StringBuilder sb = new StringBuilder(32768);
    sb.append("{\n");
    sb.append("  \"routineName\": \"").append(escapeJson(routineName)).append("\",\n");
    sb.append("  \"robotName\": \"").append(escapeJson(robotName)).append("\",\n");
    sb.append("  \"fieldMode\": \"EMPTY_FIELD_2027\",\n");
    if (autoWarning != null && !autoWarning.isBlank()) {
      sb.append("  \"autoWarning\": \"").append(escapeJson(autoWarning)).append("\",\n");
    }
    sb.append("  \"fps\": 25,\n");
    sb.append("  \"floorBalls\": [],\n");
    sb.append("  \"shots\": [],\n");
    sb.append("  \"frames\": [\n");
    for (int i = 0; i < frames.size(); i++) {
      AutoFrame f = frames.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"t\":%.3f,\"x\":%.4f,\"y\":%.4f,\"h\":%.4f,\"vx\":%.3f,\"vy\":%.3f,\"om\":%.3f,\"elev\":%.3f,\"arm\":%.3f,\"rol\":%.2f,\"vBat\":%.2f,\"iDrv\":%.1f,\"scale\":%.2f,\"st\":\"%s\"}",
              f.timeSec(),
              f.xMeters(),
              f.yMeters(),
              f.headingRad(),
              f.vxMps(),
              f.vyMps(),
              f.omegaRadPerSec(),
              f.elevatorHeightM(),
              f.armAngleRad(),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.accelScale(),
              escapeJson(f.stateLabel())));
      if (i + 1 < frames.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ]\n}\n");
    Files.writeString(jsonFile, sb.toString(), StandardCharsets.UTF_8);
  }

  private static String escapeJson(String s) {
    return (s == null) ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"");
  }
}
