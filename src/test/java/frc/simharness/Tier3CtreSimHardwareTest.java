package frc.simharness;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.drive.Drive;
import frc.lib.drive.GyroIO.GyroIOInputs;
import frc.lib.drive.GyroIOPigeon2;
import frc.lib.drive.ModuleIOTalonFX;
import frc.lib.drive.SwerveConfig;
import frc.lib.drive.SwerveConfig.CornerConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO.MechanismIOInputs;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.util.PhoenixSignals;
import java.nio.file.Paths;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tier 3 Verification Gate: Real CTRE Phoenix 6 C++ Hardware Simulation (libCTRE_SimProTalonFX.so)
 * + WPILib DCMotorSim exercising {@link MechanismIOTalonFX} directly.
 *
 * <p>Verifies that applied TalonFXConfiguration registers enforce 60A Stator / 40A Supply limits
 * (preventing the NYTR 120A SlipCurrent brownout bug) and tests 250 Hz closed-loop velocity
 * tracking precision.
 */
public class Tier3CtreSimHardwareTest {
  private static TalonFX keepAliveTalon;

  @BeforeAll
  public static void initHal() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL must initialize in headless Linux container");
    SimHooks.stepTiming(0.020);
    keepAliveTalon = new TalonFX(62, new CANBus("rio"));
    Unmanaged.feedEnable(100);
    Thread.sleep(200);
  }

  @BeforeEach
  public void settleCanBusBetweenTests() throws InterruptedException {
    Unmanaged.feedEnable(100);
    Thread.sleep(100);
  }

  @AfterAll
  public static void closeKeepAlive() {
    if (keepAliveTalon != null) {
      keepAliveTalon.close();
      keepAliveTalon = null;
    }
  }

  @Test
  public void testMechanismIoTalonFxConfigReadbackAndSlipCurrentGuard() {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec swerveSpec = fig.getMechanismSpec("FUEL", "SwerveDriveFL");
    MechanismSpec rioSpec =
        new MechanismSpec(
            swerveSpec.robot(),
            swerveSpec.mechanism(),
            31,
            "rio",
            swerveSpec.statorLimitAmps(),
            swerveSpec.supplyLimitAmps(),
            swerveSpec.slipCurrentAmps(),
            swerveSpec.gravityType(),
            swerveSpec.kP(),
            swerveSpec.kI(),
            swerveSpec.kD(),
            swerveSpec.kS(),
            swerveSpec.kV(),
            swerveSpec.kA(),
            swerveSpec.kG(),
            swerveSpec.minPosRot(),
            swerveSpec.maxPosRot());

    try (MechanismIOTalonFX io = new MechanismIOTalonFX(rioSpec)) {
      TalonFXConfiguration readback = io.readAppliedConfig();

      assertTrue(readback.CurrentLimits.StatorCurrentLimitEnable);
      assertTrue(readback.CurrentLimits.SupplyCurrentLimitEnable);

      PrecisionReporter.recordExactTarget(
          3,
          "tier3_talonfx_stator_limit_amps",
          "CTRE TalonFX Stator Limit Readback",
          readback.CurrentLimits.StatorCurrentLimit,
          60.0,
          0.1,
          "A");
      PrecisionReporter.recordExactTarget(
          3,
          "tier3_talonfx_supply_limit_amps",
          "CTRE TalonFX Supply Limit Readback",
          readback.CurrentLimits.SupplyCurrentLimit,
          40.0,
          0.1,
          "A");

      assertEquals(60.0, readback.CurrentLimits.StatorCurrentLimit, 0.1);
      assertEquals(40.0, readback.CurrentLimits.SupplyCurrentLimit, 0.1);
    }

    // Verify that attempting the NYTR 120A SlipCurrent overwrite is caught immediately
    MechanismSpec buggyNytrSpec =
        new MechanismSpec(
            "FUEL", "BuggySwerve", 1, "rio", 60.0, 40.0, 120.0, "NONE",
            0.22, 0.0, 0.0, 0.14, 0.12, 0.015, 0.0, -1000.0, 1000.0);
    assertThrows(
        IllegalArgumentException.class,
        () -> new MechanismIOTalonFX(buggyNytrSpec),
        "MechanismIOTalonFX must reject SlipCurrent > 60A");
  }

  @Test
  public void testClosedLoopKrakenX60VelocityTrackingAndStatorClampInCtreSim()
      throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    MechanismSpec baseSpec = fig.getMechanismSpec("COMP_2027", "SwerveDriveFL");
    MechanismSpec velocitySpec =
        new MechanismSpec(
            baseSpec.robot(),
            baseSpec.mechanism(),
            41,
            "rio",
            baseSpec.statorLimitAmps(),
            baseSpec.supplyLimitAmps(),
            baseSpec.slipCurrentAmps(),
            baseSpec.gravityType(),
            baseSpec.kP(),
            baseSpec.kI(),
            baseSpec.kD(),
            baseSpec.kS(),
            baseSpec.kV(),
            baseSpec.kA(),
            baseSpec.kG(),
            baseSpec.minPosRot(),
            baseSpec.maxPosRot());

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    try (MechanismIOTalonFX io = new MechanismIOTalonFX(velocitySpec)) {
      TalonFXSimState simState = io.getMotor().getSimState();
      simState.setSupplyVoltage(12.0);

      DCMotor kraken = DCMotor.getKrakenX60Foc(1);
      DCMotorSim plant =
          new DCMotorSim(LinearSystemId.createDCMotorSystem(kraken, 0.0015, 1.0), kraken);

      MechanismIOInputs inputs = new MechanismIOInputs();
      double targetRps = 65.0;

      double peakStatorAmps = 0.0;
      double dt = 0.004; // 250 Hz physics loop
      for (int step = 0; step < 250; step++) {
        Unmanaged.feedEnable(100);
        io.setVelocity(targetRps);
        Thread.sleep(2);

        double motorVoltage = simState.getMotorVoltage();
        plant.setInputVoltage(motorVoltage);
        plant.update(dt);

        double posRot = plant.getAngularPositionRotations();
        double velRps = plant.getAngularVelocityRPM() / 60.0;
        simState.setRawRotorPosition(posRot);
        simState.setRotorVelocity(velRps);

        PhoenixSignals.refreshAll();
        io.updateInputs(inputs);
        peakStatorAmps = Math.max(peakStatorAmps, Math.abs(inputs.statorCurrentA));
      }

      double finalVelRps = inputs.velocityRPS;
      double trackingErrRps = Math.abs(finalVelRps - targetRps);

      PrecisionReporter.recordLowerIsBetter(
          3,
          "tier3_kraken_velocity_tracking_err_rps",
          "Kraken X60 Sim Velocity Error",
          trackingErrRps,
          0.05,
          1.50,
          "RPS");
      PrecisionReporter.recordLowerIsBetter(
          3,
          "tier3_kraken_peak_stator_current_amps",
          "Kraken X60 Sim Peak Stator Current",
          peakStatorAmps,
          45.0,
          61.0,
          "A");

      assertTrue(
          trackingErrRps < 1.50,
          "Kraken X60 closed-loop velocity error must be < 1.5 RPS (measured: "
              + trackingErrRps
              + ")");
      assertTrue(
          peakStatorAmps <= 61.0,
          "Kraken X60 stator current must respect 60A limit (measured: " + peakStatorAmps + ")");
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  public void testModuleIoTalonFxAndGyroIoPigeon2Readback() throws InterruptedException {
    FluxFig fig = new FluxFig(Paths.get("src/main/deploy/configs"));
    SwerveConfig baseCfg = SwerveConfig.fromFluxFig(fig, "FUEL");
    SwerveConfig rioCfg =
        new SwerveConfig(
            "rio",
            baseCfg.pigeonId(),
            baseCfg.trackWidthMeters(),
            baseCfg.wheelBaseMeters(),
            baseCfg.wheelRadiusMeters(),
            baseCfg.driveGearRatio(),
            baseCfg.turnGearRatio(),
            baseCfg.maxLinearSpeedMps(),
            baseCfg.maxAngularSpeedRadPerSec(),
            baseCfg.driveStatorLimitAmps(),
            baseCfg.driveSupplyLimitAmps(),
            baseCfg.driveSlipCurrentAmps(),
            baseCfg.turnStatorLimitAmps(),
            baseCfg.turnSupplyLimitAmps(),
            baseCfg.driveKp(),
            baseCfg.driveKi(),
            baseCfg.driveKd(),
            baseCfg.driveKs(),
            baseCfg.driveKv(),
            baseCfg.driveKa(),
            baseCfg.turnKp(),
            baseCfg.turnKd(),
            baseCfg.corners());

    try (ModuleIOTalonFX modFl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[0]);
        ModuleIOTalonFX modFr = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[1]);
        ModuleIOTalonFX modBl = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[2]);
        ModuleIOTalonFX modBr = new ModuleIOTalonFX(rioCfg, rioCfg.corners()[3]);
        GyroIOPigeon2 gyroIo = new GyroIOPigeon2(55, "rio")) {
      TalonFXConfiguration flDriveCfg = modFl.readAppliedDriveConfig();
      TalonFXConfiguration frDriveCfg = modFr.readAppliedDriveConfig();
      assertTrue(flDriveCfg.CurrentLimits.StatorCurrentLimitEnable);
      assertTrue(flDriveCfg.CurrentLimits.SupplyCurrentLimitEnable);
      assertEquals(60.0, flDriveCfg.CurrentLimits.StatorCurrentLimit, 0.1);
      assertEquals(40.0, flDriveCfg.CurrentLimits.SupplyCurrentLimit, 0.1);

      // Verify left vs. right drive motor inversion and non-zero CANcoder magnet offset on hardware
      CANcoderConfiguration flCancoderCfg = new CANcoderConfiguration();
      modFl.getCancoder().getConfigurator().refresh(flCancoderCfg, 0.100);

      // Verify end-to-end Drive odometry integration with real ModuleIOTalonFX + PhoenixOdometryThread
      // WITHOUT manually calling PhoenixOdometryThread.sampleNow() from the test!
      Drive hardwareDrive = new Drive(rioCfg, gyroIo, modFl, modFr, modBl, modBr);
      hardwareDrive.setPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
      ModuleIOTalonFX[] allMods = {modFl, modFr, modBl, modBr};
      double rotorPos = 0.0;
      for (int cycle = 0; cycle < 20; cycle++) {
        Unmanaged.feedEnable(100);
        rotorPos += 1.25; // advance wheel rotation
        for (ModuleIOTalonFX m : allMods) {
          m.getDriveTalon().getSimState().setSupplyVoltage(12.2);
          m.getDriveTalon().getSimState().setRawRotorPosition(rotorPos);
          m.getDriveTalon().getSimState().setRotorVelocity(62.5);
        }
        Thread.sleep(6);
        PhoenixSignals.refreshAll();
        hardwareDrive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0));
        hardwareDrive.periodic();
      }
      PrecisionReporter.recordHigherIsBetter(
          3,
          "tier3_hardware_odometry_samples",
          "Real ModuleIOTalonFX Odometry Samples",
          hardwareDrive.getOdometrySampleCount(),
          20.0,
          20.0,
          "samples");

      assertEquals(
          InvertedValue.CounterClockwise_Positive,
          flDriveCfg.MotorOutput.Inverted,
          "FL drive motor must be CounterClockwise_Positive");
      assertEquals(
          InvertedValue.Clockwise_Positive,
          frDriveCfg.MotorOutput.Inverted,
          "FR drive motor must be Clockwise_Positive (right-side bevel gear inversion)");
      assertEquals(
          0.105225,
          flCancoderCfg.MagnetSensor.MagnetOffset,
          1e-3,
          "FL CANcoder hardware configurator must apply calibrated magnet offset");
      assertTrue(
          hardwareDrive.getOdometrySampleCount() >= 20,
          "Drive + ModuleIOTalonFX must integrate >= 20 odometry samples automatically (actual="
              + hardwareDrive.getOdometrySampleCount()
              + ")");
      assertTrue(
          hardwareDrive.getPose().getX() > 0.20 && hardwareDrive.getPose().getX() < 2.00,
          "Drive pose X must advance in [0.20m, 2.00m] on real ModuleIOTalonFX hardware IOs without 2*pi radian inflation (actual="
              + hardwareDrive.getPose().getX()
              + ")");

      GyroIOInputs gyroInputs = new GyroIOInputs();
      PhoenixSignals.refreshAll();
      gyroIo.updateInputs(gyroInputs);
    }
  }
}
