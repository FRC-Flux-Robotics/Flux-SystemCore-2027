package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utility methods for CTRE Phoenix 6 hardware configuration with a bounded boot wall-clock budget
 * so offline/faulted CAN buses never stall robotInit() across multiple devices.
 */
public final class PhoenixUtil {
  /** Default total wall-clock budget (seconds) shared across boot-time CTRE config retries. */
  public static final double DEFAULT_CAN_CONFIG_BUDGET_SECONDS = 3.0;

  /** Bounded per-call CTRE configurator timeout (seconds). */
  public static final double DEFAULT_CAN_CALL_TIMEOUT_SECONDS = 0.100;

  private static final long RETRY_SLEEP_MS = 25L;

  private static volatile double configBudgetSeconds = DEFAULT_CAN_CONFIG_BUDGET_SECONDS;
  private static volatile long budgetStartNanos = System.nanoTime();
  private static volatile int skippedCalls = 0;
  private static volatile int failedCalls = 0;
  private static volatile boolean budgetWarningReported = false;

  private PhoenixUtil() {}

  /**
   * Retries a CTRE configuration call until it succeeds, max attempts are exhausted, or the shared
   * boot {@code CanConfigBudget} expires.
   */
  public static StatusCode tryUntilOk(Supplier<StatusCode> configCall, int maxAttempts) {
    return tryUntilOk("CTREConfig", configCall, maxAttempts);
  }

  /**
   * Retries a named CTRE configuration call under the shared boot {@code CanConfigBudget}. When the
   * budget is exhausted, skips immediately without blocking boot.
   */
  public static StatusCode tryUntilOk(
      String label, Supplier<StatusCode> configCall, int maxAttempts) {
    if (getRemainingConfigBudgetSeconds() <= 0.0) {
      skippedCalls++;
      reportBudgetExhaustedOnce(label);
      return StatusCode.TxTimeout;
    }

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < maxAttempts; i++) {
      if (getRemainingConfigBudgetSeconds() <= 0.0) {
        skippedCalls++;
        reportBudgetExhaustedOnce(label);
        return StatusCode.TxTimeout;
      }
      status = configCall.get();
      if (status.isOK()) {
        return status;
      }
      if (i + 1 < maxAttempts && getRemainingConfigBudgetSeconds() > 0.0) {
        try {
          Thread.sleep(RETRY_SLEEP_MS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          break;
        }
      }
    }
    if (!status.isOK()) {
      failedCalls++;
    }
    return status;
  }

  public static double getRemainingConfigBudgetSeconds() {
    double elapsedSec = (System.nanoTime() - budgetStartNanos) / 1e9;
    return Math.max(0.0, configBudgetSeconds - elapsedSec);
  }

  public static int getSkippedConfigCallCount() {
    return skippedCalls;
  }

  public static int getFailedConfigCallCount() {
    return failedCalls;
  }

  /** Resets the shared boot config budget (intended for isolated unit/sim test suites). */
  public static void resetConfigBudgetForTest() {
    resetConfigBudgetForTest(DEFAULT_CAN_CONFIG_BUDGET_SECONDS);
  }

  public static void resetConfigBudgetForTest(double budgetSeconds) {
    configBudgetSeconds = Math.max(0.0, budgetSeconds);
    budgetStartNanos = System.nanoTime();
    skippedCalls = 0;
    failedCalls = 0;
    budgetWarningReported = false;
  }

  private static void reportBudgetExhaustedOnce(String label) {
    if (!budgetWarningReported) {
      budgetWarningReported = true;
      DriverStation.reportWarning(
          "PhoenixUtil CAN config budget ("
              + configBudgetSeconds
              + " s) exhausted before completing '"
              + label
              + "'; skipping remaining retries.",
          false);
    }
  }
}
