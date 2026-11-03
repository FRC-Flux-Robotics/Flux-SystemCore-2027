package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Live-editable dashboard tunable number (Team 6328 pattern) backed by AdvantageKit {@link
 * LoggedNetworkNumber}. Automatically locks to its default value when {@code tuningMode == false}
 * or when connected to FMS at competition.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";
  private static boolean tuningMode = false;

  private final String key;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Double testOverride = null;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    key = TABLE_KEY + "/" + dashboardKey;
    initDefault(defaultValue);
  }

  /** Globally enables or disables live dashboard tuning (always disabled when FMS is attached). */
  public static void setTuningMode(boolean enabled) {
    tuningMode = enabled;
  }

  public static boolean isTuningMode() {
    return tuningMode && !DriverStation.isFMSAttached();
  }

  public void initDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (isTuningMode() && dashboardNumber == null) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  public void setTestOverride(double value) {
    testOverride = value;
  }

  public void clearTestOverride() {
    testOverride = null;
  }

  public double get() {
    if (!isTuningMode()) {
      return defaultValue;
    }
    if (testOverride != null) {
      return testOverride;
    }
    if (dashboardNumber == null) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
    return dashboardNumber.get();
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || Double.compare(currentValue, lastValue) != 0) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    boolean changed = false;
    for (LoggedTunableNumber num : tunableNumbers) {
      if (num.hasChanged(id)) {
        changed = true;
      }
    }
    if (changed) {
      double[] values = new double[tunableNumbers.length];
      for (int i = 0; i < tunableNumbers.length; i++) {
        values[i] = tunableNumbers[i].get();
      }
      action.accept(values);
    }
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    boolean changed = false;
    for (LoggedTunableNumber num : tunableNumbers) {
      if (num.hasChanged(id)) {
        changed = true;
      }
    }
    if (changed) {
      action.run();
    }
  }
}
