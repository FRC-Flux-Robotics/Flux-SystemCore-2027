package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.lib.safety.FaultSupervisor;
import frc.lib.vision.VisionIOSim;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure.State;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tier 4 Verification Gate: Headless 50x Fast-Forward 150-Second Full Match Simulation with
 * 5-Scenario {@link FaultSupervisor} Chaos Fault Injection and Binary {@code .wpilog} Recording.
 */
class Tier4ChaosMatchSimTest {

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void testFull150SecChaosMatchFastForwardZeroBrownouts() throws Exception {
    SimHooks.pauseTiming();
    try {
      RobotContainer container =
          new RobotContainer(RobotIdentity.COMP_2027, Paths.get("src/main/deploy/configs"), true);
      FaultSupervisor supervisor = container.getFaultSupervisor();
      Path wpilogPath = Path.of("build/chaos-match-sim.wpilog");

      // Pre-match disabled calibration phase: 60 loops of stationary MegaTag1 observations
      for (int d = 0; d < 60; d++) {
        SimHooks.stepTiming(0.020);
        container.getVision().periodic();
        container.disabledPeriodic();
      }
      assertTrue(
          container.getPreMatchCalibrator().isGyroSeeded(),
          "PreMatchCalibrator must seed gyro during 60 disabled loops before match start");

      double weakBatteryRintOhms = 0.028; // 28 mOhm degraded competition battery
      double openCircuitVoltage = 12.4;
      double minBatteryVoltageV = 12.4;
      int brownoutCount = 0;
      int quarantineDetectedLoop = -1;
      int bentCamInjectLoop = 1100; // t = 22.0s

      try (FluxWpiLogCodec.Writer log =
          new FluxWpiLogCodec.Writer(wpilogPath, "FLUX-2027-Tier4-ChaosMatch")) {
        int vBatEntry = log.startEntry("/Robot/BatteryVoltageV", "double", "V", 0);
        int statorLimitEntry =
            log.startEntry("/FaultSupervisor/ActiveDriveStatorLimitA", "double", "A", 0);
        int cam1QuarantinedEntry =
            log.startEntry("/FaultSupervisor/Cam1Quarantined", "boolean", "", 0);
        int dejamCountEntry = log.startEntry("/FaultSupervisor/DejamEvents", "double", "count", 0);

        // 7,500 loops * 0.020s = 150.0s full FRC match (15s Auto + 135s Teleop/Endgame)
        for (int loop = 0; loop < 7500; loop++) {
          SimHooks.stepTiming(0.020);
          long tsUs = loop * 20_000L;
          double timeSec = loop * 0.020;

          if (loop == bentCamInjectLoop) {
            ((VisionIOSim) container.getVision().getCameraIo(1))
                .setMountErrorPerturbation(
                    new Transform3d(new Translation3d(0.12, 0.0, 0.0), new Rotation3d()));
          }

          boolean jamActive = (loop >= 2400 && loop <= 2420) || (loop >= 4800 && loop <= 4820);
          double indexerCurrentA = jamActive ? 45.0 : 12.0;
          double indexerVelRps = jamActive ? 0.10 : 32.0;
          double indexerCmdV =
              supervisor.filterMechanismCommandVoltage(8.0, indexerCurrentA, indexerVelRps);

          int visibleTags = (loop >= 3500 && loop <= 3650) ? 0 : 2;
          for (int c = 0; c < 4; c++) {
            ((VisionIOSim) container.getVision().getCameraIo(c))
                .setVisibleTagSimulation(visibleTags, 2.6, 0.08);
          }

          double speedScale = supervisor.reportSteerEncoderFault(2, loop >= 5750);

          double activeStatorLimitA =
              supervisor.updateBatteryProtection(minBatteryVoltageV, weakBatteryRintOhms * 1000.0);
          double demandFactor =
              0.65 + 0.35 * Math.abs(Math.sin(timeSec * 1.7)) * speedScale;
          double totalDriveSupplyA = 4.0 * activeStatorLimitA * demandFactor * 0.62;
          double mechSupplyA = (indexerCmdV < 0.0) ? 14.0 : 22.0;
          double totalSupplyA = totalDriveSupplyA + mechSupplyA;

          double batteryVoltageV = openCircuitVoltage - totalSupplyA * weakBatteryRintOhms;
          supervisor.updateBatteryProtection(batteryVoltageV, weakBatteryRintOhms * 1000.0);

          if (batteryVoltageV < minBatteryVoltageV) {
            minBatteryVoltageV = batteryVoltageV;
          }
          if (batteryVoltageV < 6.80) {
            brownoutCount++;
          }

          // Step actual RobotContainer subsystems during the match simulation
          container.getDrive().runVelocity(new ChassisSpeeds(2.5 * demandFactor, 0.0, 0.5));
          container.getSuperstructure().requestState((loop % 200 < 100) ? State.SCORING_HIGH : State.STOWED);
          container.getDrive().periodic();
          container.getVision().periodic();
          if (quarantineDetectedLoop < 0 && supervisor.isCameraQuarantined(1)) {
            quarantineDetectedLoop = loop;
          }
          container.getElevator().periodic();
          container.getArmPivot().periodic();
          container.getIntakeRoller().periodic();
          container.getSuperstructure().periodic();
          container.periodicUpdate();

          if (loop % 10 == 0) {
            log.appendDouble(vBatEntry, tsUs, batteryVoltageV);
            log.appendDouble(statorLimitEntry, tsUs, supervisor.activeDriveStatorLimitAmps());
            log.appendBoolean(cam1QuarantinedEntry, tsUs, supervisor.isCameraQuarantined(1));
            log.appendDouble(dejamCountEntry, tsUs, supervisor.dejamEventsTriggered());
          }
        }
      }

      double quarantineLatencyMs = (quarantineDetectedLoop - bentCamInjectLoop) * 20.0;
      FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(wpilogPath);

      PrecisionReporter.recordHigherIsBetter(
          4,
          "tier4_chaos_min_vbat",
          "150s Chaos Match Min Battery V",
          minBatteryVoltageV,
          9.50,
          7.50,
          "V");
      PrecisionReporter.recordLowerIsBetter(
          4,
          "tier4_cam_quarantine_ms",
          "Bent Cam (+12cm) Quarantine Latency",
          quarantineLatencyMs,
          80.0,
          200.0,
          "ms");
      PrecisionReporter.recordExactTarget(
          4,
          "tier4_indexer_dejam_pulses",
          "Indexer Stall Jam Auto-Clears",
          supervisor.dejamEventsTriggered(),
          2.0,
          0.1,
          "pulses");

      assertEquals(0, brownoutCount, "Zero battery brownouts (<6.8V) allowed during 150s chaos match");
      assertTrue(minBatteryVoltageV >= 7.50, "Battery voltage must stay >= 7.50V on 28mOhm battery");
      assertTrue(supervisor.isCameraQuarantined(1), "Bent camera #1 (+12cm) must be quarantined");
      assertEquals(80.0, quarantineLatencyMs, 25.0, "Quarantine must trigger in 5 frames (~80-100ms)");
      assertEquals(2, supervisor.dejamEventsTriggered(), "Both injected indexer jams must auto-clear");
      assertTrue(Files.size(wpilogPath) > 10_000, "Generated .wpilog must contain full match trace");
      assertTrue(parsed.durationSeconds() >= 149.0, "Parsed .wpilog duration must span ~150s match");
    } finally {
      SimHooks.resumeTiming();
    }
  }
}
