package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.lib.drive.GyroIOPigeon2;
import frc.lib.drive.ModuleIO;
import frc.lib.drive.ModuleIOSim;
import frc.lib.drive.ModuleIOTalonFX;
import frc.lib.drive.SwerveConfig;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.CameraSpec;
import frc.lib.fluxfig.FluxFig.MechanismSpec;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIOSim;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.safety.BatteryIO;
import frc.lib.safety.FaultSupervisor;
import frc.lib.util.FieldPositions;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PiecewiseSensitivity;
import frc.lib.util.VirtualBatterySim;
import frc.lib.vision.Vision;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIOLimelightSystemCore;
import frc.lib.vision.VisionIOPhotonVision;
import frc.lib.vision.VisionIOSim;
import frc.robot.Superstructure.State;
import frc.robot.autos.StudentAutoRoutines;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Clean 2027 Pre-Kickoff robot container (zero 2026 shooting bloat) wiring 250 Hz {@link Drive},
 * full-queue {@link Vision}, {@link BatteryIO} (`Playing With Fusion BFG` on {@code auxCanBus}),
 * {@link PreMatchCalibrator}, {@link Superstructure} (`Elevator` + `ArmPivot` + `IntakeRoller` with
 * 2D keep-out &amp; CG anti-tipping), Ghost Superstructure hybrid mode, and shared {@code
 * frc.lib.commands.*} bindings.
 */
public class RobotContainer {
  /**
   * Unified top-level robot operating mode:
   *
   * <ul>
   *   <li>{@link #COMPETITION}: Default ({@code FluxFig.Mode.PRODUCTION}) and automatically forced
   *       whenever connected to FMS ({@code DriverStation.isFMSAttached()}). Locks {@link FluxFig}
   *       and {@link LoggedTunableNumber} to verified Git/CSV values, forces {@link
   *       FieldPositions.FieldProfile#FULL_FIELD} (geofence OFF), and disables pit/blitz test
   *       button bindings while on FMS.
   *   <li>{@link #TRAINING}: Active in shop/practice when {@code FluxFig.Mode.TUNING} is enabled
   *       and not connected to FMS. Enables live dashboard tunables, custom/auto-measured shop or
   *       half-field geofence bounds, and 1-button pit/blitz routines.
   * </ul>
   */
  public enum OperatingMode {
    COMPETITION,
    TRAINING
  }

  private final RobotIdentity identity;
  private final FluxFig fluxFig;
  private final FaultSupervisor faultSupervisor;
  private final BatteryIO batteryIo;
  private final VirtualBatterySim virtualBatterySim;
  private final PiecewiseSensitivity joystickSensitivity;

  private final Drive drive;
  private final Vision vision;
  private final PreMatchCalibrator preMatchCalibrator;

  private final PositionMechanism elevator;
  private final PositionMechanism armPivot;
  private final VelocityMechanism intakeRoller;
  private final Superstructure superstructure;
  private final boolean ghostSuperstructureActive;
  private final boolean forceSimIo;
  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer(RobotIdentity identity) {
    this(
        identity,
        Filesystem.getDeployDirectory().toPath().resolve("configs"),
        !RobotBase.isReal());
  }

  /** Deterministic constructor allowing explicit config directory and simulated IO selection. */
  public RobotContainer(RobotIdentity identity, Path configDir, boolean forceSimIo) {
    this.identity = identity;
    this.forceSimIo = forceSimIo;
    this.fluxFig = FluxFig.load(configDir, identity.name());

    String coordSysName = fluxFig.getString("coordinateSystem");
    FieldPositions.setCoordinateSystem(
        "WALL_BLUE_2026".equalsIgnoreCase(coordSysName)
            ? FieldPositions.CoordinateSystem.WALL_BLUE_2026
            : FieldPositions.CoordinateSystem.CENTER_RED_2027);
    applyFieldAndArenaConfig();

    String profileRobot =
        (identity == RobotIdentity.FUEL || identity == RobotIdentity.CORAL)
            ? identity.name()
            : RobotIdentity.COMP_2027.name();

    boolean simMissingMechs = fluxFig.getBoolean("simulateMissingMechanisms");
    this.ghostSuperstructureActive =
        simMissingMechs
            || identity == RobotIdentity.CORAL
            || identity == RobotIdentity.CHASSIS_3;

    CameraSpec[] camConfigs =
        fluxFig.getCamerasForRobot(profileRobot).toArray(new CameraSpec[0]);
    this.faultSupervisor = new FaultSupervisor(Math.max(1, camConfigs.length));
    this.batteryIo = new BatteryIO(fluxFig, fluxFig.getString("auxCanBus"));
    this.virtualBatterySim = new VirtualBatterySim(12.6, 0.015);
    this.joystickSensitivity =
        new PiecewiseSensitivity("Drive/JoystickCurve", 0.05, 0.60, 0.08, 0.45, 1.0);

    SwerveConfig swerveConfig = SwerveConfig.fromFluxFig(fluxFig, identity.name());
    GyroIO gyroIo;
    ModuleIO[] moduleIos = new ModuleIO[4];
    if (forceSimIo) {
      gyroIo = new GyroIO() {};
      for (int i = 0; i < 4; i++) {
        moduleIos[i] = new ModuleIOSim(swerveConfig);
      }
    } else {
      gyroIo = new GyroIOPigeon2(swerveConfig.pigeonId(), swerveConfig.canBus());
      for (int i = 0; i < 4; i++) {
        moduleIos[i] = new ModuleIOTalonFX(swerveConfig, swerveConfig.corners()[i]);
      }
    }
    this.drive = new Drive(swerveConfig, gyroIo, moduleIos);
    this.drive.setLoopPeriodSeconds(fluxFig.getInt("loopPeriodMs") / 1000.0);

    VisionIO[] visionIos = new VisionIO[camConfigs.length];
    String visionBackend = fluxFig.getString("visionBackend");
    if (forceSimIo) {
      for (int i = 0; i < camConfigs.length; i++) {
        visionIos[i] = new VisionIOSim(camConfigs[i], drive::getPose);
      }
    } else if ("LIMELIGHT_SYSTEMCORE_USB".equalsIgnoreCase(visionBackend)) {
      for (int i = 0; i < camConfigs.length; i++) {
        visionIos[i] = new VisionIOLimelightSystemCore(camConfigs[i]);
      }
    } else {
      AprilTagFieldLayout fieldLayout =
          FieldPositions.configureLayoutOrigin(
              AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
      for (int i = 0; i < camConfigs.length; i++) {
        visionIos[i] = new VisionIOPhotonVision(camConfigs[i], fieldLayout);
      }
    }
    this.vision = new Vision(drive::addVisionMeasurement, faultSupervisor, camConfigs, visionIos);
    this.preMatchCalibrator = new PreMatchCalibrator(drive, vision);

    boolean useSimMechanisms = forceSimIo || ghostSuperstructureActive;
    String mechProfile =
        (identity == RobotIdentity.COMP_2027) ? fluxFig.getActiveProfile() : "COMP_2027";

    MechanismSpec elevSpec = fluxFig.getMechanismSpec(mechProfile, "ElevatorLeader");
    MechanismIO elevIo =
        useSimMechanisms ? new MechanismIOSim(elevSpec) : new MechanismIOTalonFX(elevSpec);
    this.elevator = new PositionMechanism(elevIo, elevSpec);

    MechanismSpec armSpec = fluxFig.getMechanismSpec(mechProfile, "ArmPivot");
    MechanismIO armIo =
        useSimMechanisms ? new MechanismIOSim(armSpec) : new MechanismIOTalonFX(armSpec);
    this.armPivot = new PositionMechanism(armIo, armSpec);

    MechanismSpec rollerSpec = fluxFig.getMechanismSpec(mechProfile, "IntakeRoller");
    MechanismIO rollerIo =
        useSimMechanisms ? new MechanismIOSim(rollerSpec) : new MechanismIOTalonFX(rollerSpec);
    this.intakeRoller = new VelocityMechanism(rollerIo, rollerSpec);

    this.superstructure =
        new Superstructure(this.elevator, this.armPivot, this.intakeRoller, this.faultSupervisor);

    registerNamedCommands();
    this.autoChooser = new LoggedDashboardChooser<>("Auto/Chooser");
    configureAutoChooser();
    configureButtonBindings();

    Logger.recordOutput("FluxFig/EffectiveVersion", fluxFig.getEffectiveVersion());
    Logger.recordOutput("FluxFig/EffectiveSnapshot", fluxFig.getEffectiveSnapshot());
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "intake",
        Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure));
    NamedCommands.registerCommand(
        "stow", Commands.runOnce(() -> superstructure.requestState(State.STOWED), superstructure));
    NamedCommands.registerCommand(
        "scoreLow",
        Commands.runOnce(() -> superstructure.requestState(State.SCORING_LOW), superstructure));
    NamedCommands.registerCommand(
        "scoreHigh",
        Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH), superstructure));
    NamedCommands.registerCommand(
        "eject",
        Commands.runOnce(() -> superstructure.requestState(State.EJECTING), superstructure));
    NamedCommands.registerCommand("homeElevator", new HardStopHomingCmd(elevator));
  }

  private void configureAutoChooser() {
    var namedAutos = StudentAutoRoutines.createNamedAutos(drive, superstructure);
    boolean firstAdded = false;
    for (var entry : namedAutos.entrySet()) {
      if (!firstAdded) {
        autoChooser.addDefaultOption(entry.getKey(), entry.getValue());
        firstAdded = true;
      } else {
        autoChooser.addOption(entry.getKey(), entry.getValue());
      }
    }
    autoChooser.addOption(
        "Center DriveToPose (1.0m)", new DriveToPose(drive, new Pose2d(1.0, 0.0, Rotation2d.kZero)));
    autoChooser.addOption("Pit System Check (20s)", createPitSystemCheckCommand());
    autoChooser.addOption(
        "15-Min Floor Blitz", createBlitz15MinCommand(Blitz15MinCmd.DEFAULT_CHECKPOINT_PATH));
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            joystickSensitivity,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis() > 0.5));

    driverController
        .a()
        .whileTrue(
            new DriveToPose(
                drive,
                () -> new Pose2d(0.0, 0.0, Rotation2d.kZero),
                () -> driverController.getLeftTriggerAxis() > 0.5));
    driverController.b().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .start()
        .onTrue(createPitSystemCheckCommand().onlyIf(() -> !DriverStation.isFMSAttached()));
    driverController
        .back()
        .whileTrue(
            createBlitz15MinCommand(Blitz15MinCmd.DEFAULT_CHECKPOINT_PATH)
                .onlyIf(() -> !DriverStation.isFMSAttached()));

    operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.INTAKING), superstructure))
        .onFalse(Commands.runOnce(() -> superstructure.requestState(State.STOWED), superstructure));
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.SCORING_LOW), superstructure));
    operatorController
        .y()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.SCORING_HIGH), superstructure));
    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> superstructure.requestState(State.STOWED), superstructure));
    operatorController.start().onTrue(new HardStopHomingCmd(elevator));
  }

  public PitSystemCheckCmd createPitSystemCheckCommand() {
    return new PitSystemCheckCmd(
        drive, List.of(elevator, armPivot), List.of(intakeRoller));
  }

  public Blitz15MinCmd createBlitz15MinCommand(Path checkpointPath) {
    BooleanSupplier deadManHeld = () -> forceSimIo || driverController.getLeftTriggerAxis() > 0.5;
    return new Blitz15MinCmd(
        createPitSystemCheckCommand(),
        new HardStopHomingCmd(elevator),
        new ZeroPriorBootstrapCmd(drive, vision, deadManHeld),
        new ShopStressSuiteCmd(drive, deadManHeld),
        checkpointPath);
  }

  public OperatingMode getOperatingMode() {
    if (DriverStation.isFMSAttached() || fluxFig.getMode() != FluxFig.Mode.TUNING) {
      return OperatingMode.COMPETITION;
    }
    return OperatingMode.TRAINING;
  }

  private void applyFieldAndArenaConfig() {
    double shopLen = fluxFig.hasParameter("shopLengthMeters") ? fluxFig.getDouble("shopLengthMeters") : 5.0;
    double shopWid = fluxFig.hasParameter("shopWidthMeters") ? fluxFig.getDouble("shopWidthMeters") : 3.0;
    FieldPositions.setShopDimensions(shopLen, shopWid);
    if (fluxFig.hasParameter("fieldProfile")) {
      try {
        FieldPositions.setFieldProfile(
            FieldPositions.FieldProfile.valueOf(fluxFig.getString("fieldProfile").trim()));
      } catch (IllegalArgumentException ignored) {
        FieldPositions.setFieldProfile(FieldPositions.FieldProfile.FULL_FIELD);
      }
    }
  }

  /**
   * Executes passive MegaTag1 gyro seeding, starting pose audit, and {@code < 50 ms} SFTP {@code
   * active-profile.txt} hot-reload while Disabled.
   */
  public void disabledPeriodic() {
    boolean fms = DriverStation.isFMSAttached();
    fluxFig.lockProductionOnFms(fms);
    if (!fms && fluxFig.reloadIfChanged()) {
      applyFieldAndArenaConfig();
      Logger.recordOutput("FluxFig/EffectiveVersion", fluxFig.getEffectiveVersion());
      Logger.recordOutput("FluxFig/EffectiveSnapshot", fluxFig.getEffectiveSnapshot());
    }
    preMatchCalibrator.updateDisabled();
  }

  /**
   * Executes container-level coordination every loop: updates {@link BatteryIO}, scales swerve
   * acceleration based on elevator CG height, and feeds simulated/real mechanism currents into
   * {@link VirtualBatterySim} + {@link FaultSupervisor}.
   */
  public void periodicUpdate() {
    boolean fms = DriverStation.isFMSAttached();
    fluxFig.lockProductionOnFms(fms);
    OperatingMode mode = getOperatingMode();
    boolean training = mode == OperatingMode.TRAINING;
    LoggedTunableNumber.setTuningMode(training);
    if (training) {
      fluxFig.pollTuning();
    }
    Logger.recordOutput("Robot/OperatingMode", mode.name());

    joystickSensitivity.pollTunables();
    batteryIo.periodic();

    vision.setRobotOrientation(
        drive.getPose().getRotation(),
        drive.getChassisSpeeds().omegaRadiansPerSecond,
        preMatchCalibrator.isGyroSeeded(),
        drive.getMaxTiltRad());

    double accelScale = superstructure.getSafeSwerveAccelerationScale();
    drive.setMaxAccelerationScale(accelScale);

    double ghostSupplyAmps =
        ghostSuperstructureActive ? superstructure.getTotalMechanismSupplyCurrentAmps() : 0.0;
    double simVBat =
        virtualBatterySim.update(drive.getTotalSupplyCurrentAmps(), ghostSupplyAmps);
    double effectiveVBat =
        (!forceSimIo && RobotBase.isReal() && !ghostSuperstructureActive)
            ? RobotController.getBatteryVoltage()
            : simVBat;
    double effectiveRintMohm =
        batteryIo.isHealthy()
            ? batteryIo.getInternalResistanceMilliOhms()
            : virtualBatterySim.estimatedRintMilliOhms();
    faultSupervisor.updateBatteryProtection(effectiveVBat, effectiveRintMohm);
    drive.applyBatteryProtection(faultSupervisor);

    Logger.recordOutput("Superstructure/GhostModeActive", ghostSuperstructureActive);
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    return (selected != null)
        ? selected
        : StudentAutoRoutines.createStudentDefaultAuto(drive, superstructure);
  }

  /**
   * Resolves a named autonomous routine by exact or case-insensitive substring match for {@code
   * ./gradlew renderAuto -Pauto="<name>"}.
   */
  public Command resolveAutonomousCommand(String selector) {
    if (selector == null || selector.isBlank()) {
      return getAutonomousCommand();
    }
    String needle = selector.trim().toLowerCase(java.util.Locale.US);
    var namedAutos = StudentAutoRoutines.createNamedAutos(drive, superstructure);
    for (var entry : namedAutos.entrySet()) {
      if (entry.getKey().equalsIgnoreCase(selector.trim())) {
        return entry.getValue().withName(entry.getKey());
      }
    }
    for (var entry : namedAutos.entrySet()) {
      if (entry.getKey().toLowerCase(java.util.Locale.US).contains(needle)) {
        return entry.getValue().withName(entry.getKey());
      }
    }
    throw new IllegalArgumentException(
        "Unknown autonomous routine '"
            + selector
            + "'. Available routines: "
            + String.join(" | ", namedAutos.keySet()));
  }

  public Command createExampleAutonomousCommand() {
    return StudentAutoRoutines.createExampleSquareAndCircleAuto(drive, superstructure);
  }

  public LoggedDashboardChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public RobotIdentity getIdentity() {
    return identity;
  }

  public FluxFig getFluxFig() {
    return fluxFig;
  }

  public FaultSupervisor getFaultSupervisor() {
    return faultSupervisor;
  }

  public BatteryIO getBatteryIo() {
    return batteryIo;
  }

  public VirtualBatterySim getVirtualBatterySim() {
    return virtualBatterySim;
  }

  public PiecewiseSensitivity getJoystickSensitivity() {
    return joystickSensitivity;
  }

  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }

  public PreMatchCalibrator getPreMatchCalibrator() {
    return preMatchCalibrator;
  }

  public PositionMechanism getElevator() {
    return elevator;
  }

  public PositionMechanism getArmPivot() {
    return armPivot;
  }

  public VelocityMechanism getIntakeRoller() {
    return intakeRoller;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public boolean isGhostSuperstructureActive() {
    return ghostSuperstructureActive;
  }
}
