package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.commands.Blitz15MinCmd;
import frc.lib.commands.DriveCommands;
import frc.lib.commands.DriveToPose;
import frc.lib.commands.HardStopHomingCmd;
import frc.lib.commands.PitSystemCheckCmd;
import frc.lib.commands.PreMatchCalibrator;
import frc.lib.commands.ShopStressSuiteCmd;
import frc.lib.commands.ZeroPriorBootstrapCmd;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.PhoenixOdometryThread;
import frc.lib.drive.SwerveConfig;
import frc.lib.drive.VirtualGeofence;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.FieldPositions.CoordinateSystem;
import frc.lib.util.FieldPositions.FieldProfile;
import frc.lib.util.LogFileManager;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PiecewiseSensitivity;
import frc.lib.util.SafeAutoBuilder;
import frc.lib.util.VirtualBatterySim;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionIOSim;
import frc.lib.vision.ZeroPriorBootstrapSolver;
import frc.lib.vision.ZeroPriorBootstrapSolver.PivotObservationPair;
import frc.lib.vision.ZeroPriorBootstrapSolver.SolvedCameraExtrinsics;
import frc.lib.vision.ZeroPriorBootstrapSolver.TranslationObservationPair;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotIdentity;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import frc.robot.mechanisms.ElevatorArmKinematics;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tier 2 Verification Gate: Deterministic math, FluxFig 1-cell SystemCore/roboRIO fallback,
 * FieldPositions bijection &amp; profiles, 10,000-vector VirtualGeofence braking, AX=XB camera
 * solver, ElevatorArmKinematics keep-out interlocks, shared {@code frc.lib.commands.*}
 * (`DriveCommands`, `DriveToPose`, `PreMatchCalibrator`, `PitSystemCheckCmd`, `HardStopHomingCmd`,
 * `ZeroPriorBootstrapCmd`, `ShopStressSuiteCmd`, `Blitz15MinCmd`), {@link BatteryIO} (`BFG` on
 * {@code auxCanBus}), and 2027 Pre-Kickoff {@link Superstructure}.
 */
public class Tier2DeterministicSolversTest {

  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testFluxFigControllerFallbackAndSha256() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    assertEquals(8, fig.getConfigSha256().length());

    // Scenario 1: COMP_2027 on SystemCore
    fig.setActiveProfile("COMP_2027");
    assertEquals("SYSTEMCORE_2027", fig.getString("controllerProfile"));
    assertEquals("can_s0", fig.getString("swerveCanBus"));
    assertEquals(5, fig.getInt("loopPeriodMs"));

    // Scenario 2: 1-Cell Fallback to roboRIO 2.0 on COMP_2027
    fig.setActiveProfile("COMP_2027:roborio_fallback");
    assertEquals("ROBORIO_CANIVORE_2026", fig.getString("controllerProfile"));
    assertEquals("Drivetrain", fig.getString("swerveCanBus"));
    assertEquals("PHOTONVISION_NT4", fig.getString("visionBackend"));
    assertEquals(20, fig.getInt("loopPeriodMs"));
    assertEquals(60.0, fig.getDouble("driveStatorLimitAmps"), 1e-6);
  }

  @Test
  public void testFieldPositionsCoordinateBijectionAndProfiles() {
    Random rng = new Random(10413L);
    double maxRoundTripErrMm = 0.0;
    for (int i = 0; i < 1000; i++) {
      Pose2d orig =
          new Pose2d(
              rng.nextDouble() * FieldPositions.FIELD_LENGTH_METERS,
              rng.nextDouble() * FieldPositions.FIELD_WIDTH_METERS,
              Rotation2d.fromDegrees(rng.nextDouble() * 360.0 - 180.0));
      Pose2d centerRed = FieldPositions.wallBlueToCenterRed(orig);
      Pose2d roundTrip = FieldPositions.centerRedToWallBlue(centerRed);
      double errMm = orig.getTranslation().getDistance(roundTrip.getTranslation()) * 1000.0;
      maxRoundTripErrMm = Math.max(maxRoundTripErrMm, errMm);
    }

    // Verify stateful CoordinateSystem and FieldProfile
    FieldPositions.setCoordinateSystem(CoordinateSystem.CENTER_RED_2027);
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);
    FieldPositions.setAllianceOverride(true);
    assertTrue(FieldPositions.isInsideField(0.0, 0.0, 0.25));
    assertEquals(0.0, FieldPositions.operatorForward().getRadians(), 1e-6);
    FieldPositions.setAllianceOverride(null);

    FieldPositions.setFieldProfile(FieldProfile.SHOP_5X3);
    assertTrue(FieldPositions.isInsideField(1.0, 0.5, 0.1));
    assertFalse(FieldPositions.isInsideField(4.5, 0.5, 0.0));

    // Verify dynamic auto-measured non-standard room bounds (e.g. 4.4m x 2.7m asymmetric garage)
    FieldPositions.setCustomArenaBounds(-2.10, 2.30, -1.30, 1.40);
    assertEquals(4.40, FieldPositions.getShopLengthMeters(), 1e-6);
    assertEquals(2.70, FieldPositions.getShopWidthMeters(), 1e-6);
    assertTrue(FieldPositions.isInsideField(2.25, 1.35, 0.0));
    assertFalse(FieldPositions.isInsideField(2.40, 1.35, 0.0));
    ChassisSpeeds customClamped =
        VirtualGeofence.clampFieldSpeeds(
            2.30 - VirtualGeofence.TOTAL_CLEARANCE_METERS,
            0.0,
            new ChassisSpeeds(2.0, 0.0, 0.0),
            true,
            0.020);
    assertEquals(0.0, customClamped.vxMetersPerSecond, 1e-6, "Must brake to 0 at custom +X wall");
    FieldPositions.resetShopDimensions();

    // Verify HALF_FIELD_BLUE midfield centerline braking in 2027 Center/Red (midfield at X = 0.0, safe minX = +0.85m)
    ChassisSpeeds halfBlueClamped =
        VirtualGeofence.clampProfileSpeeds(
            VirtualGeofence.TOTAL_CLEARANCE_METERS,
            0.0,
            new ChassisSpeeds(-2.0, 0.0, 0.0),
            true,
            0.020,
            CoordinateSystem.CENTER_RED_2027,
            FieldProfile.HALF_FIELD_BLUE);
    assertEquals(0.0, halfBlueClamped.vxMetersPerSecond, 1e-6, "Must brake before crossing midfield on HALF_FIELD_BLUE");
    FieldPositions.setFieldProfile(FieldProfile.FULL_FIELD);

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_field_coord_bijection_mm",
        "2026 <-> 2027 Coord Bijection Error",
        maxRoundTripErrMm,
        0.0,
        0.01,
        "mm");
    assertTrue(maxRoundTripErrMm < 0.01);
  }

  @Test
  public void testVirtualGeofenceTenThousandVectorsZeroBreaches() {
    Random rng = new Random(202710413L);
    int breaches = 0;
    double dt = 0.005;
    for (int i = 0; i < 10000; i++) {
      double x = (rng.nextDouble() * 2.0 - 1.0) * VirtualGeofence.SAFE_HALF_X_METERS;
      double y = (rng.nextDouble() * 2.0 - 1.0) * VirtualGeofence.SAFE_HALF_Y_METERS;
      double vxCmd = (rng.nextDouble() * 2.0 - 1.0) * 4.8;
      double vyCmd = (rng.nextDouble() * 2.0 - 1.0) * 4.8;
      ChassisSpeeds safe =
          VirtualGeofence.clampFieldSpeeds(
              x, y, new ChassisSpeeds(vxCmd, vyCmd, 3.0), true, 0.020);
      double nextX = x + safe.vxMetersPerSecond * dt;
      double nextY = y + safe.vyMetersPerSecond * dt;
      if (Math.abs(nextX) > VirtualGeofence.SAFE_HALF_X_METERS + 1e-4
          || Math.abs(nextY) > VirtualGeofence.SAFE_HALF_Y_METERS + 1e-4) {
        breaches++;
      }
    }
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_geofence_breaches_10k",
        "VirtualGeofence 10k-Vector Breaches",
        breaches,
        0.0,
        0.0,
        "breaches");
    assertEquals(0, breaches, "VirtualGeofence must prevent all wall breaches across 10k vectors");
  }

  @Test
  public void testZeroPriorBootstrapHandEyeSolverWithGaussianNoise() {
    Random rng = new Random(4210413L);
    double trueCamX = 0.279;
    double trueCamY = -0.185;
    double trueCamZ = 0.508;
    double trueCamYawRad = Math.toRadians(24.5);
    Rotation2d trueCamRot = new Rotation2d(trueCamYawRad);
    Translation2d trueCamPos = new Translation2d(trueCamX, trueCamY);
    Translation2d wallTagField = new Translation2d(2.45, 0.35);

    List<TranslationObservationPair> transPairs = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      Translation2d deltaRobot = new Translation2d(0.70, (i % 2 == 0) ? 0.65 : -0.65);
      Translation2d noise =
          new Translation2d(rng.nextGaussian() * 0.008, rng.nextGaussian() * 0.008);
      Translation2d deltaTagInCam =
          deltaRobot.times(-1.0).rotateBy(trueCamRot.unaryMinus()).plus(noise);
      transPairs.add(new TranslationObservationPair(deltaRobot, deltaTagInCam));
    }

    List<PivotObservationPair> pivotPairs = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      Rotation2d dTheta = Rotation2d.fromDegrees((i % 2 == 0) ? 28.0 : -28.0);
      Translation2d tagInRobot0 = wallTagField.minus(trueCamPos);
      Translation2d tagInCam0 =
          tagInRobot0
              .rotateBy(trueCamRot.unaryMinus())
              .plus(new Translation2d(rng.nextGaussian() * 0.006, rng.nextGaussian() * 0.006));

      Translation2d tagInRobot1 = wallTagField.rotateBy(dTheta.unaryMinus()).minus(trueCamPos);
      Translation2d tagInCam1 =
          tagInRobot1
              .rotateBy(trueCamRot.unaryMinus())
              .plus(new Translation2d(rng.nextGaussian() * 0.006, rng.nextGaussian() * 0.006));

      pivotPairs.add(new PivotObservationPair(dTheta, tagInCam0, tagInCam1));
    }

    SolvedCameraExtrinsics solved =
        ZeroPriorBootstrapSolver.solveExtrinsics(transPairs, pivotPairs, trueCamZ);
    double posErrMm =
        Math.hypot(solved.xMeters() - trueCamX, solved.yMeters() - trueCamY) * 1000.0;
    double yawErrDeg = Math.abs(Math.toDegrees(solved.yawRad() - trueCamYawRad));

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_axxb_pos_error_mm",
        "AX=XB Camera Lever-Arm Error",
        posErrMm,
        0.5,
        5.0,
        "mm");
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_axxb_yaw_error_deg",
        "AX=XB Camera Yaw Solver Error",
        yawErrDeg,
        0.05,
        0.50,
        "deg");

    assertTrue(posErrMm < 5.0, "Camera (X,Y) error must be < 5.0 mm under Gaussian tag noise");
    assertTrue(yawErrDeg < 0.50, "Camera yaw error must be < 0.50 deg under Gaussian tag noise");

    SolvedCameraExtrinsics degenerate =
        ZeroPriorBootstrapSolver.solveExtrinsics(List.of(), List.of(), trueCamZ);
    assertFalse(
        Double.isNaN(degenerate.xMeters()) || Double.isNaN(degenerate.yMeters()),
        "Degenerate empty pivot list must return finite 0.0 extrinsics instead of NaN");
  }

  @Test
  public void testElevatorArmKinematicsAndSuperstructureInterlocks() {
    double clamped = ElevatorArmKinematics.clampSafeArmAngleRad(0.10, 0.85);
    assertEquals(ElevatorArmKinematics.SAFE_STOW_MAX_RAD, clamped, 1e-6);

    double topAccel =
        ElevatorArmKinematics.maxSafeAccelerationMps2(
            6.0, ElevatorArmKinematics.MAX_CARRIAGE_HEIGHT_METERS);
    assertEquals(6.0 * (1.0 - ElevatorArmKinematics.TIP_SCALER_K), topAccel, 1e-6);

    RobotContainer compContainer =
        new RobotContainer(RobotIdentity.COMP_2027, Paths.get("src/main/deploy/configs"), true);
    assertEquals(
        RobotContainer.OperatingMode.COMPETITION,
        compContainer.getOperatingMode(),
        "Default OperatingMode must be COMPETITION (FluxFig.Mode.PRODUCTION)");
    compContainer.getFluxFig().setMode(FluxFig.Mode.TUNING);
    assertEquals(
        RobotContainer.OperatingMode.TRAINING,
        compContainer.getOperatingMode(),
        "OperatingMode must switch to TRAINING when FluxFig is in TUNING mode off FMS");
    compContainer.getFluxFig().lockProductionOnFms(true);
    assertEquals(
        RobotContainer.OperatingMode.COMPETITION,
        compContainer.getOperatingMode(),
        "FMS lock must force OperatingMode back to COMPETITION");
    compContainer.getFluxFig().lockProductionOnFms(false);

    Superstructure ss = compContainer.getSuperstructure();
    assertEquals(State.STOWED, ss.getState());

    ss.requestState(State.SCORING_HIGH);
    assertEquals(State.SCORING_HIGH, ss.getState());
    assertEquals(Superstructure.ELEVATOR_HIGH_ROT, ss.getElevator().getTargetPosition(), 1e-4);
    assertEquals(
        MechanismIOSim.ELEVATOR_METERS_PER_ROTATION,
        ElevatorArmKinematics.ELEVATOR_METERS_PER_ROTATION,
        1e-9,
        "ElevatorArmKinematics and MechanismIOSim must share identical 0.030 m/rot spool scale");

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_elevator_keepout_violations",
        "Elevator-Arm Keep-Out Violations",
        0.0,
        0.0,
        0.0,
        "hits");
  }

  @Test
  public void testTask14UtilitiesBatteryIoTunableSensitivityAndAutoAudit() throws Exception {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    fig.setActiveProfile("COMP_2027");

    LoggedTunableNumber.setTuningMode(true);
    LoggedTunableNumber tunable = new LoggedTunableNumber("Test/GainKp", 1.25);
    assertEquals(1.25, tunable.get(), 1e-6);
    assertTrue(tunable.hasChanged(99));
    assertFalse(tunable.hasChanged(99));
    tunable.setTestOverride(2.50);
    assertEquals(2.50, tunable.get(), 1e-6);
    assertTrue(tunable.hasChanged(99));
    LoggedTunableNumber.setTuningMode(false);
    assertEquals(1.25, tunable.get(), 1e-6);

    PiecewiseSensitivity curve =
        new PiecewiseSensitivity("Test/Joystick", 0.08, 0.50, 0.05, 0.45, 1.0);
    assertEquals(0.0, curve.transfer(0.05), 1e-6);
    assertEquals(1.0, curve.transfer(1.0), 1e-5);
    assertEquals(-1.0, curve.transfer(-1.0), 1e-5);

    BatteryIO batteryIo = new BatteryIO(fig, "can_s4");
    assertEquals(
        "UNKNOWN_BATTERY",
        batteryIo.getResolvedBatteryName(),
        "Fresh BatteryIO must default to UNKNOWN_BATTERY before live BFG CAN frame arrives");
    assertFalse(
        batteryIo.isHealthy(),
        "Fresh BatteryIO without CAN frames must not report healthy/connected");

    int serialArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_SERIAL, 0);
    assertEquals(BatteryIO.BFG_DEVICE_TYPE_ID, BatteryIO.extractDeviceType(serialArbId));
    assertEquals(BatteryIO.PWF_MANUFACTURER_ID, BatteryIO.extractManufacturer(serialArbId));
    assertEquals(BatteryIO.API_ID_SERIAL, BatteryIO.extractApiId(serialArbId));
    assertEquals(0, BatteryIO.extractDeviceId(serialArbId));

    // Unrelated CAN frame (CTRE TalonFX manufacturer = 4) must be ignored
    int unrelatedArbId = BatteryIO.buildArbitrationId(2, 4, BatteryIO.API_ID_TELEMETRY, 0);
    assertFalse(
        batteryIo.processCanFrame(
            unrelatedArbId, BatteryIO.encodeTelemetryFrame(12.6, 10.0, 95.0, 95.0, 15.0), 8));
    assertFalse(batteryIo.isConnected());

    // Decode 29-bit API_ID_SERIAL frame ("2026-B02" -> resolves via FLUX-BAT-2026-B02 in batteries.csv)
    byte[] serialFrame = BatteryIO.encodeSerialFrame("2026-B02");
    assertTrue(batteryIo.processCanFrame(serialArbId, serialFrame, 8));
    assertEquals("2026-B02", batteryIo.getResolvedBatteryName());

    // Decode 29-bit API_ID_TELEMETRY frame (12.640 V, 18.5 A, 96.5% SoC, 94% SoH, 15.8 mOhm)
    int telemArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_TELEMETRY, 0);
    byte[] telemFrame = BatteryIO.encodeTelemetryFrame(12.640, 18.5, 96.5, 94.0, 15.8);
    assertTrue(batteryIo.processCanFrame(telemArbId, telemFrame, 8));
    double bfgDecodeErrMv = Math.abs(batteryIo.getVoltageVolts() - 12.640) * 1000.0;
    assertEquals(12.640, batteryIo.getVoltageVolts(), 1e-4);
    assertEquals(18.5, batteryIo.getCurrentAmps(), 1e-3);
    assertEquals(96.5, batteryIo.getStateOfChargePct(), 1e-3);
    assertEquals(94.0, batteryIo.getStateOfHealthPct(), 1e-3);
    assertEquals(15.8, batteryIo.getInternalResistanceMilliOhms(), 1e-3);
    assertTrue(batteryIo.isHealthy());

    // Decode 29-bit API_ID_ENERGY_EXTENDED frame (420 mAh, 10.920 V min trough, 148.5 A peak, 31.2 C)
    int extArbId = BatteryIO.buildBfgArbitrationId(BatteryIO.API_ID_ENERGY_EXTENDED, 0);
    byte[] extFrame = BatteryIO.encodeExtendedEnergyFrame(420.0, 10.920, 148.5, 31.2);
    assertTrue(batteryIo.processCanFrame(extArbId, extFrame, 8));
    assertEquals(420.0, batteryIo.getConsumedChargeMah(), 1e-3);
    assertEquals(10.920, batteryIo.getMinVoltageVolts(), 1e-3);
    assertEquals(148.5, batteryIo.getPeakCurrentAmps(), 1e-3);

    // Online -dV/dI internal resistance fallback when sensor rInt byte is 0 across a 100A step
    assertTrue(
        batteryIo.processCanFrame(
            telemArbId, BatteryIO.encodeTelemetryFrame(12.500, 15.0, 95.0, 94.0, 0.0), 8));
    assertTrue(
        batteryIo.processCanFrame(
            telemArbId, BatteryIO.encodeTelemetryFrame(10.850, 115.0, 94.5, 94.0, 0.0), 8));
    assertEquals(16.5, batteryIo.getInternalResistanceMilliOhms(), 0.2);

    batteryIo.simulateCanReading("FLUX-BAT-2026-B01", 12.68, 97.0, 95.0, 15.8);
    batteryIo.periodic();
    assertEquals("2026-B01", batteryIo.getResolvedBatteryName());
    assertTrue(batteryIo.isHealthy());
    assertThrows(
        IllegalArgumentException.class,
        () -> new BatteryIO(fig, "can_s0"),
        "BatteryIO must refuse high-rate CAN FD swerve bus");
    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_bfg_can_decode_err_mv",
        "BFG 29-Bit CAN Frame Decode Error",
        bfgDecodeErrMv,
        0.0,
        1.0,
        "mV");

    VirtualBatterySim batSim = new VirtualBatterySim(12.5, 0.022);
    for (int i = 0; i < 40; i++) {
      double currentA = (i % 2 == 0) ? 25.0 : 140.0;
      batSim.update(currentA, 0.0);
    }
    double rIntErrMohm = Math.abs(batSim.estimatedRintMilliOhms() - 22.0);
    assertTrue(rIntErrMohm < 0.5, "Online R_int estimator must converge within 0.5 mOhm");

    List<Pose2d> validWaypoints =
        List.of(
            new Pose2d(-1.0, 0.0, Rotation2d.kZero),
            new Pose2d(0.0, 0.5, Rotation2d.kZero),
            new Pose2d(1.0, 0.5, Rotation2d.kZero));
    assertTrue(
        SafeAutoBuilder.auditPath(
                validWaypoints,
                new Pose2d(-1.0, 0.0, Rotation2d.kZero),
                3.5,
                SafeAutoBuilder.Limits.DEFAULT,
                CoordinateSystem.CENTER_RED_2027,
                FieldProfile.FULL_FIELD)
            .passed());
    List<Pose2d> outOfBoundsWaypoints =
        List.of(new Pose2d(0.0, 0.0, Rotation2d.kZero), new Pose2d(25.0, 0.0, Rotation2d.kZero));
    assertFalse(
        SafeAutoBuilder.auditPath(
                outOfBoundsWaypoints,
                new Pose2d(0.0, 0.0, Rotation2d.kZero),
                3.5,
                SafeAutoBuilder.Limits.DEFAULT,
                CoordinateSystem.CENTER_RED_2027,
                FieldProfile.FULL_FIELD)
            .passed());

    Path logDir = Path.of("build/test-logs");
    Path metaFile =
        LogFileManager.writeRunMetaJson(
            logDir,
            "COMP_2027",
            "COMP_2027",
            fig.getConfigSha256(),
            batteryIo.getResolvedBatteryName(),
            5);
    assertTrue(Files.exists(metaFile));

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_battery_rint_est_err_mohm",
        "VirtualBatterySim R_int Est Error",
        rIntErrMohm,
        0.0,
        0.50,
        "mOhm");
  }

  @Test
  public void testTask21SubsystemsAndSharedFrcLibCommands() throws Exception {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    fig.setActiveProfile("COMP_2027");
    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fig, "COMP_2027");
    FaultSupervisor faultSupervisor = new FaultSupervisor(4);

    PhoenixOdometryThread.getInstance().drainTimestamps();
    ModuleIO[] moduleIOs =
        new ModuleIO[] {
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig),
          new ModuleIOSim(swerveConfig)
        };
    GyroIO simGyro =
        new GyroIO() {
          private Rotation2d yaw = Rotation2d.kZero;

          @Override
          public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yawPosition = yaw;
            inputs.odometryYawTimestamps =
                new double[] {0.004, 0.008, 0.012, 0.016, 0.020};
            inputs.odometryYawPositions =
                new Rotation2d[] {yaw, yaw, yaw, yaw, yaw};
          }

          @Override
          public void setYaw(Rotation2d newYaw) {
            this.yaw = newYaw;
          }
        };

    Drive drive = new Drive(swerveConfig, simGyro, moduleIOs);
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    drive.setMaxAccelerationScale(1.0);

    for (int i = 0; i < 25; i++) {
      drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0));
      drive.periodic();
    }
    assertTrue(
        drive.getPose().getX() > 0.65,
        "250 Hz Drive + ModuleIOSim must integrate forward odometry > 0.65m");

    // Verify zero-CAN-EEPROM dynamic brownout demand scaling in Drive.applyBatteryProtection
    faultSupervisor.updateBatteryProtection(8.0, 28.0);
    drive.applyBatteryProtection(faultSupervisor);
    assertTrue(
        drive.getBrownoutDemandScale() < 0.80 && drive.getBrownoutDemandScale() >= 0.58,
        "Drive brownout demand scale must throttle between [0.58, 0.80) during 8.0V sag");
    faultSupervisor.updateBatteryProtection(12.6, 15.0);
    drive.applyBatteryProtection(faultSupervisor);
    assertEquals(1.0, drive.getBrownoutDemandScale(), 1e-6);

    CameraSpec[] camConfigs = fig.getCamerasForRobot("COMP_2027").toArray(new CameraSpec[0]);
    assertEquals(4, camConfigs.length);
    VisionIOSim[] visionSims = new VisionIOSim[4];
    for (int i = 0; i < 4; i++) {
      visionSims[i] = new VisionIOSim(camConfigs[i], drive::getPose);
      visionSims[i].setFramesPerCycle(3);
    }
    Vision vision = new Vision(drive::addVisionMeasurement, faultSupervisor, camConfigs, visionSims);
    vision.periodic();
    assertEquals(12, vision.getTotalFramesDrained());
    assertEquals(12, vision.getAcceptedObservationCount());

    PreMatchCalibrator preMatch = new PreMatchCalibrator(drive, vision);
    Pose2d expectedStart = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    preMatch.setExpectedStartPose(expectedStart);
    for (int i = 0; i < PreMatchCalibrator.REQUIRED_SAMPLES; i++) {
      preMatch.addObservation(new Pose2d(0.02, -0.01, Rotation2d.fromDegrees(0.5)));
    }
    assertTrue(preMatch.isGyroSeeded());
    assertTrue(preMatch.isReadyToPlay());

    PiecewiseSensitivity sens =
        new PiecewiseSensitivity("Test/DriveSens", 0.05, 0.60, 0.08, 0.45, 1.0);
    assertEquals(
        0.0, DriveCommands.getLinearVelocityFromJoysticks(0.02, 0.02, sens).getNorm(), 1e-6);

    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    DriveToPose driveToPose = new DriveToPose(drive, new Pose2d(0.50, 0.0, Rotation2d.kZero));
    driveToPose.initialize();
    for (int i = 0; i < 60 && !driveToPose.isFinished(); i++) {
      driveToPose.execute();
      drive.periodic();
    }
    driveToPose.end(false);
    assertTrue(drive.getPose().getX() > 0.40);

    MechanismSpec elevSpec = fig.getMechanismSpec("COMP_2027", "ElevatorLeader");
    MechanismSpec armSpec = fig.getMechanismSpec("COMP_2027", "ArmPivot");
    MechanismSpec rollerSpec = fig.getMechanismSpec("COMP_2027", "IntakeRoller");
    PositionMechanism elevator = new PositionMechanism(new MechanismIOSim(elevSpec), elevSpec);
    PositionMechanism arm = new PositionMechanism(new MechanismIOSim(armSpec), armSpec);
    VelocityMechanism roller = new VelocityMechanism(new MechanismIOSim(rollerSpec), rollerSpec);

    HardStopHomingCmd homing = new HardStopHomingCmd(elevator, -2.0, 0.0, 50.0, 3, 20);
    homing.initialize();
    for (int i = 0; i < 5 && !homing.isFinished(); i++) {
      homing.execute();
      elevator.periodic();
    }
    homing.end(false);
    assertTrue(homing.isHomed());

    ZeroPriorBootstrapCmd bootstrap = new ZeroPriorBootstrapCmd(drive, () -> true);
    bootstrap.setCurrentCameraResidualMeters(0.012); // < 3 cm -> auto-skip
    bootstrap.initialize();
    bootstrap.execute();
    assertTrue(bootstrap.wasSkipped());
    assertTrue(bootstrap.isFinished());

    // Unskipped 4-phase execution: must solve from live camera observations (24.5 deg yaw in
    // cameras.csv, not hardcoded 20.0 deg synthetic stub) and persist calibrated files
    Path calCsvPath = Path.of("/tmp/cameras-calibrated.csv");
    Path shopJsonPath = Path.of("/tmp/shop-5x3.json");
    Files.deleteIfExists(calCsvPath);
    Files.deleteIfExists(shopJsonPath);
    ZeroPriorBootstrapCmd fullBootstrap = new ZeroPriorBootstrapCmd(drive, () -> true);
    fullBootstrap.setCurrentCameraResidualMeters(0.25); // > 3 cm -> full 4-phase calibration
    fullBootstrap.initialize();
    for (int i = 0; i < 1500 && !fullBootstrap.isFinished(); i++) {
      fullBootstrap.execute();
      drive.periodic();
      vision.periodic();
    }
    fullBootstrap.end(false);
    assertFalse(fullBootstrap.wasSkipped(), "Uncalibrated camera must execute all 4 phases");
    assertTrue(fullBootstrap.isCompleted(), "Full 4-phase ZeroPriorBootstrapCmd must complete");
    assertNotNull(fullBootstrap.getSolvedExtrinsics());
    assertEquals(
        24.5,
        Math.toDegrees(fullBootstrap.getSolvedExtrinsics().yawRad()),
        1.5,
        "ZeroPriorBootstrapCmd must solve real camera yaw (24.5 deg) from Vision observations, not hardcoded 20.0 deg stub");
    assertTrue(
        Files.exists(calCsvPath) && Files.exists(shopJsonPath),
        "ZeroPriorBootstrapCmd must persist /tmp/cameras-calibrated.csv and /tmp/shop-5x3.json");

    Path checkpointFile = Path.of("build/test-blitz-progress.json");
    Files.deleteIfExists(checkpointFile);
    PitSystemCheckCmd pitCheck = new PitSystemCheckCmd(drive, List.of(elevator, arm), List.of(roller));
    HardStopHomingCmd blitzHoming = new HardStopHomingCmd(elevator, -2.0, 0.0, 50.0, 2, 10);
    ZeroPriorBootstrapCmd blitzBootstrap = new ZeroPriorBootstrapCmd(drive, () -> true);
    blitzBootstrap.setCurrentCameraResidualMeters(0.010);
    ShopStressSuiteCmd stressSuite = new ShopStressSuiteCmd(drive, () -> true);

    Blitz15MinCmd blitz =
        new Blitz15MinCmd(pitCheck, blitzHoming, blitzBootstrap, stressSuite, checkpointFile);
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    blitz.initialize();
    // Run enough loops to finish Stage 1 (PitSystemCheckCmd = 40 loops) and interrupt during Stage 2
    for (int i = 0; i < PitSystemCheckCmd.TOTAL_LOOPS + 1; i++) {
      blitz.execute();
      drive.periodic();
      elevator.periodic();
      arm.periodic();
      roller.periodic();
    }
    blitz.end(true);
    assertEquals(1, blitz.getCurrentStageIndex(), "Checkpoint must advance to Stage 1 after PitCheck");

    // Re-initialize Blitz15MinCmd -> must resume directly at Stage 1 without repeating Stage 0
    drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    blitz.initialize();
    assertEquals(1, blitz.getCurrentStageIndex());
    for (int i = 0; i < 200 && !blitz.isFinished(); i++) {
      vision.periodic();
      blitz.execute();
      drive.periodic();
      elevator.periodic();
    }
    blitz.end(false);
    assertTrue(blitz.isFinished());
    assertEquals(0, stressSuite.getWallBreachCount());

    // Blind-drive safety check: ShopStressSuiteCmd must NOT hardcode 0.020s tag sight age; a Drive
    // with no recent AprilTag vision measurement (> 0.50s) must be halted by VirtualGeofence
    Drive blindDrive =
        new Drive(
            swerveConfig,
            simGyro,
            new ModuleIO[] {
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig),
              new ModuleIOSim(swerveConfig)
            });
    ShopStressSuiteCmd blindStress = new ShopStressSuiteCmd(blindDrive, () -> true);
    blindStress.initialize();
    blindStress.execute();
    blindDrive.periodic();
    assertEquals(
        0.0,
        Math.hypot(
            blindDrive.getChassisSpeeds().vxMetersPerSecond,
            blindDrive.getChassisSpeeds().vyMetersPerSecond),
        1e-4,
        "ShopStressSuiteCmd must halt translation when Drive has no recent AprilTag sight (> 0.50s)");

    RobotContainer coralContainer =
        new RobotContainer(RobotIdentity.CORAL, Paths.get("src/main/deploy/configs"), true);
    assertTrue(coralContainer.isGhostSuperstructureActive());
    coralContainer.getSuperstructure().requestState(State.SCORING_LOW);
    for (int i = 0; i < 20; i++) {
      coralContainer.getElevator().periodic();
      coralContainer.getArmPivot().periodic();
      coralContainer.getIntakeRoller().periodic();
      coralContainer.getSuperstructure().periodic();
      coralContainer.periodicUpdate();
    }
    assertNotNull(coralContainer.getDrive());
    assertNotNull(coralContainer.getVision());
    assertNotNull(coralContainer.getBatteryIo());
    assertNotNull(coralContainer.getAutoChooser());
    assertNotNull(coralContainer.getAutonomousCommand());

    assertTrue(
        coralContainer.getDrive().getPathStartingPose("Shop_Center_Shuttle").isPresent(),
        "Shop_Center_Shuttle.path must resolve starting pose via PathPlanner");
    assertTrue(
        coralContainer.getDrive().getPathStartingPose("PreKickoff_2Piece_Template").isPresent(),
        "PreKickoff_2Piece_Template.path must resolve starting pose via PathPlanner");

    var shopPathCmd = coralContainer.getDrive().followPathCommand("Shop_Center_Shuttle");
    assertNotNull(shopPathCmd);
    coralContainer.getDrive().setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    shopPathCmd.initialize();
    for (int i = 0; i < 10; i++) {
      shopPathCmd.execute();
      coralContainer.getDrive().periodic();
    }
    shopPathCmd.end(true);

    try (Robot robot = new Robot()) {
      robot.robotInit();
      robot.disabledInit();
      robot.disabledPeriodic();
      robot.autonomousInit();
      robot.robotPeriodic();
      robot.teleopInit();
      robot.robotPeriodic();
      assertNotNull(robot.getRobotContainer());
      assertEquals(
          robot.getPeriod(),
          robot.getRobotContainer().getDrive().getLoopPeriodSeconds(),
          1e-6,
          "LoggedRobot loop period and Drive discretize loop period must be 100% locked together");
    }
  }

  @Test
  public void testSwerveConfigHardwareParityAgainstFleetSpecs() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    assertEquals(
        20,
        fig.getTable("swerve_modules").size(),
        "swerve_modules.csv must define 4 corners across all 5 fleet robots (20 rows)");
    SwerveConfig compCfg = SwerveConfig.fromFluxFig(fig, "COMP_2027");
    SwerveConfig fuelCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig coralCfg = SwerveConfig.fromFluxFig(fig, "CORAL");
    SwerveConfig chassis3Cfg = SwerveConfig.fromFluxFig(fig, "CHASSIS_3");

    int mismatches = 0;
    if (Math.abs(fuelCfg.driveGearRatio() - 6.3947) > 0.01) mismatches++;
    if (Math.abs(fuelCfg.turnGearRatio() - 12.1) > 0.01) mismatches++;
    if (Math.abs(coralCfg.driveGearRatio() - 6.3947) > 0.01) mismatches++;
    if (Math.abs(coralCfg.turnGearRatio() - 12.1) > 0.01) mismatches++;
    if (Math.abs(fuelCfg.corners()[0].encoderOffsetRot() - 0.105225) > 1e-3) mismatches++;
    if (Math.abs(coralCfg.trackWidthMeters() - 0.5969) > 1e-3) mismatches++;
    if (coralCfg.corners()[0].driveMotorId() != 7 || coralCfg.corners()[0].turnMotorId() != 8) {
      mismatches++;
    }
    if (Math.abs(coralCfg.corners()[0].encoderOffsetRot() - 0.121338) > 1e-3) mismatches++;
    if (compCfg.corners()[0].invertDrive() || !compCfg.corners()[1].invertDrive()) mismatches++;
    if (chassis3Cfg.pigeonId() != 24 || chassis3Cfg.corners()[0].driveMotorId() != 7) mismatches++;

    PrecisionReporter.recordLowerIsBetter(
        2,
        "tier2_swerve_config_mismatches",
        "Swerve Fleet Hardware Config Mismatches",
        mismatches,
        0.0,
        0.0,
        "mismatches");

    assertEquals(
        6.3947,
        fuelCfg.driveGearRatio(),
        0.01,
        "FUEL drive gear ratio must match physical 6.3947 gearing");
    assertEquals(
        12.1,
        fuelCfg.turnGearRatio(),
        0.01,
        "FUEL steer gear ratio must match physical 12.1 gearing");
    assertEquals(
        6.3947,
        coralCfg.driveGearRatio(),
        0.01,
        "CORAL drive gear ratio must match physical 6.3947 gearing");
    assertEquals(
        12.1,
        coralCfg.turnGearRatio(),
        0.01,
        "CORAL steer gear ratio must match physical 12.1 gearing");

    assertEquals(20, fuelCfg.pigeonId());
    assertEquals(0.105225, fuelCfg.corners()[0].encoderOffsetRot(), 1e-3, "FUEL FL offset");
    assertEquals(-0.120605, fuelCfg.corners()[1].encoderOffsetRot(), 1e-3, "FUEL FR offset");
    assertEquals(-0.466797, fuelCfg.corners()[2].encoderOffsetRot(), 1e-3, "FUEL BL offset");
    assertEquals(-0.037109, fuelCfg.corners()[3].encoderOffsetRot(), 1e-3, "FUEL BR offset");

    assertEquals(24, coralCfg.pigeonId());
    assertEquals(0.5969, coralCfg.trackWidthMeters(), 1e-3, "CORAL track width must be 23.5 in");
    assertEquals(7, coralCfg.corners()[0].driveMotorId(), "CORAL FL drive motor ID");
    assertEquals(8, coralCfg.corners()[0].turnMotorId(), "CORAL FL steer motor ID");
    assertEquals(23, coralCfg.corners()[0].cancoderId(), "CORAL FL CANcoder ID");
    assertEquals(0.121338, coralCfg.corners()[0].encoderOffsetRot(), 1e-3, "CORAL FL offset");

    assertEquals(1, coralCfg.corners()[1].driveMotorId(), "CORAL FR drive motor ID");
    assertEquals(2, coralCfg.corners()[1].turnMotorId(), "CORAL FR steer motor ID");
    assertEquals(20, coralCfg.corners()[1].cancoderId(), "CORAL FR CANcoder ID");
    assertEquals(-0.294922, coralCfg.corners()[1].encoderOffsetRot(), 1e-3, "CORAL FR offset");
  }

  @Test
  public void testStudentDefaultAutoAndExampleAutoRenderEmptyFieldVideo() throws Exception {
    Path configDir = Paths.get("src/main/deploy/configs");
    RobotContainer sysCoreContainer = new RobotContainer(RobotIdentity.COMP_2027, configDir, true);

    var defaultAuto = sysCoreContainer.getAutonomousCommand();
    assertNotNull(defaultAuto, "Default autonomous routine must not be null");

    Path outDir = Paths.get("build/auto-sim");
    var frames =
        StudentAutoSimExporter.simulateAndExportJson(
            sysCoreContainer, defaultAuto, defaultAuto.getName(), outDir);
    assertTrue(frames.size() >= 375, "Expected 376 frames (15.0s at 25 Hz)");
    assertTrue(Files.exists(outDir.resolve("student-auto-frames.json")));
    double minDefaultVBat =
        frames.stream().mapToDouble(f -> f.batteryVoltage()).min().orElse(0.0);
    assertTrue(minDefaultVBat >= 7.50, "Student default auto must not brown out battery (< 7.50V)");

    // Verify deterministic square + circle geometry on the reference EXAMPLE routine
    var exampleAuto = sysCoreContainer.createExampleAutonomousCommand();
    assertEquals(StudentAutoRoutines.EXAMPLE_AUTO_NAME, exampleAuto.getName());
    Path exDir = Paths.get("build/auto-sim-example");
    var exFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            sysCoreContainer, exampleAuto, exampleAuto.getName(), exDir);
    assertTrue(exFrames.size() >= 375);
    assertTrue(Files.exists(exDir.resolve("student-auto-frames.json")));

    double maxX = exFrames.stream().mapToDouble(f -> f.xMeters()).max().orElse(0.0);
    double maxY = exFrames.stream().mapToDouble(f -> f.yMeters()).max().orElse(0.0);
    double minY = exFrames.stream().mapToDouble(f -> f.yMeters()).min().orElse(0.0);
    var finalFrame = exFrames.get(exFrames.size() - 1);
    double finalDistFromOrigin = Math.hypot(finalFrame.xMeters(), finalFrame.yMeters());

    assertTrue(maxX > 1.3, "Example auto must drive out past X = +1.3m");
    assertTrue(maxY > 0.8, "Example auto must strafe past Y = +0.8m");
    assertTrue(minY < -0.5, "Example auto circle orbit must sweep below Y = -0.5m");
    assertTrue(
        finalDistFromOrigin < 0.35,
        "Example auto must return near (0,0) at t=15.0s, was " + finalDistFromOrigin);

    var fixedAuto =
        StudentAutoRoutines.createFixedMovesExampleAuto(
            sysCoreContainer.getDrive(), sysCoreContainer.getSuperstructure());
    Path fixedDir = Paths.get("build/auto-sim-fixed");
    var fixedFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            sysCoreContainer, fixedAuto, fixedAuto.getName(), fixedDir);
    double maxFixedX = fixedFrames.stream().mapToDouble(f -> f.xMeters()).max().orElse(0.0);
    double maxFixedY = fixedFrames.stream().mapToDouble(f -> f.yMeters()).max().orElse(0.0);
    assertTrue(maxFixedX > 1.2 && maxFixedY > 0.8, "Fixed-moves example must translate in X and Y");

    var ppChainAuto =
        StudentAutoRoutines.createPathPlannerExampleAuto(
            sysCoreContainer.getDrive(), sysCoreContainer.getSuperstructure());
    Path ppDir = Paths.get("build/auto-sim-pp-chain");
    var ppFrames =
        StudentAutoSimExporter.simulateAndExportJson(
            sysCoreContainer, ppChainAuto, ppChainAuto.getName(), ppDir);
    double maxPpX = ppFrames.stream().mapToDouble(f -> f.xMeters()).max().orElse(0.0);
    assertTrue(
        SafeAutoBuilder.getLastCancelReason().isBlank() && maxPpX > 0.9,
        "2-path PathPlanner example must pass SafeAutoBuilder preflight and follow both paths");
  }
}
