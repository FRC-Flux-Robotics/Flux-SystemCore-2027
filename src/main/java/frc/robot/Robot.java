package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.fluxfig.FluxFig;
import frc.lib.util.LogFileManager;
import frc.lib.util.LoggedTracer;
import frc.lib.util.PhoenixSignals;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * LoggedRobot lifecycle supporting configurable 50 Hz (20 ms) and 200 Hz (5 ms) control loops via
 * FluxFig, batched CAN status signal refresh (skipped in AdvantageKit REPLAY), main-thread RT
 * priority {@code 1}, {@link CommandScheduler} execution, and per-section loop tracing.
 */
public class Robot extends LoggedRobot {
  /** Safe SCHED_FIFO priority (1 rather than 99) preventing Linux RT worker starvation (-1003). */
  public static final int MAIN_THREAD_RT_PRIORITY = 1;

  private static boolean loggerStarted = false;

  private RobotIdentity identity;
  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private String logDir;
  private String deployedGitSha = "unknown";
  private String bootTimestampIso = "";

  public Robot() {
    super(resolveBootLoopPeriodSeconds());
  }

  private static double resolveBootLoopPeriodSeconds() {
    try {
      RobotIdentity bootId = RobotIdentity.detect();
      FluxFig fig =
          FluxFig.load(
              Filesystem.getDeployDirectory().toPath().resolve("configs"), bootId.name());
      return Math.max(0.002, fig.getInt("loopPeriodMs") / 1000.0);
    } catch (Exception e) {
      return 0.020;
    }
  }

  @Override
  public void robotInit() {
    SignalLogger.enableAutoLogging(false);
    logDir = LogFileManager.getLogPath();
    bootTimestampIso = Instant.now().toString();
    deployedGitSha = LogFileManager.readDeployedGitSha(Filesystem.getDeployDirectory().toPath());
    Logger.recordMetadata("LogDirectory", logDir);
    Logger.recordMetadata("GitSHA", deployedGitSha);
    identity = RobotIdentity.detect();
    Logger.recordMetadata("RobotIdentity", identity.name());
    Logger.recordMetadata("RobotIdentitySource", RobotIdentity.getLastResolutionSource());

    if (!loggerStarted) {
      if (RobotBase.isReal()) {
        Logger.addDataReceiver(new WPILOGWriter(logDir));
        Logger.addDataReceiver(new NT4Publisher());
      }
      Logger.start();
      loggerStarted = true;
    }

    robotContainer = new RobotContainer(identity);
    robotContainer.getDrive().setLoopPeriodSeconds(getPeriod());
    Logger.recordMetadata("ConfigSHA256", robotContainer.getFluxFig().getConfigSha256());

    if (RobotBase.isReal()) {
      Threads.setCurrentThreadPriority(true, MAIN_THREAD_RT_PRIORITY);
      Logger.recordOutput("Robot/MainThreadPriority", Threads.getCurrentThreadPriority());
    }

    refreshRunMetaJson();
  }

  private void refreshRunMetaJson() {
    if (logDir == null || identity == null || robotContainer == null) {
      return;
    }
    try {
      LogFileManager.writeRunMetaJson(
          Path.of(logDir),
          identity.name(),
          robotContainer.getFluxFig().getActiveProfile(),
          robotContainer.getFluxFig().getConfigSha256(),
          robotContainer.getBatteryIo().getResolvedBatteryName(),
          robotContainer.getFluxFig().getInt("loopPeriodMs"),
          deployedGitSha,
          bootTimestampIso);
    } catch (IOException ignored) {
    }
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    if (!Logger.hasReplaySource()) {
      PhoenixSignals.refreshAll();
    }
    LoggedTracer.trace("PhoenixSignals");
    CommandScheduler.getInstance().run();
    LoggedTracer.trace("CommandScheduler");
    if (robotContainer != null) {
      robotContainer.periodicUpdate();
      LoggedTracer.trace("RobotContainer");
    }
  }

  @Override
  public void disabledInit() {
    if (robotContainer != null) {
      robotContainer.disabledPeriodic();
      refreshRunMetaJson();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (robotContainer != null) {
      robotContainer.disabledPeriodic();
    }
  }

  @Override
  public void autonomousInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    if (robotContainer != null) {
      autonomousCommand = robotContainer.getAutonomousCommand();
      if (autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(autonomousCommand);
      }
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
  }

  public RobotContainer getRobotContainer() {
    return robotContainer;
  }
}
