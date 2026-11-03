package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Lightweight per-section loop profiler that logs elapsed ms via AdvantageKit. */
public final class LoggedTracer {

  private static double lastTimestamp;

  private LoggedTracer() {}

  public static void reset() {
    lastTimestamp = Timer.getFPGATimestamp();
  }

  public static void trace(String section) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("Performance/" + section + "Ms", (now - lastTimestamp) * 1000.0);
    lastTimestamp = now;
  }
}
