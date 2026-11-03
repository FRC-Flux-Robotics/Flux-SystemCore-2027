package frc.lib.util;

import edu.wpi.first.math.MathUtil;

/**
 * Two-segment piecewise linear joystick sensitivity curve backed by {@link LoggedTunableNumber}
 * with deadzone, fine low-speed control segment, high-speed segment, and sign preservation.
 */
public class PiecewiseSensitivity {
  private double xStart;
  private double xMiddle;
  private double yStart;
  private double yMiddle;
  private double yMax;
  private double slope1;
  private double slope2;

  private final LoggedTunableNumber tunableXStart;
  private final LoggedTunableNumber tunableXMiddle;
  private final LoggedTunableNumber tunableYStart;
  private final LoggedTunableNumber tunableYMiddle;
  private final LoggedTunableNumber tunableYMax;

  public PiecewiseSensitivity(
      double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
    tunableXStart = null;
    tunableXMiddle = null;
    tunableYStart = null;
    tunableYMiddle = null;
    tunableYMax = null;
    set(xStart, xMiddle, yStart, yMiddle, yMax);
  }

  public PiecewiseSensitivity(
      String keyPrefix,
      double xStart,
      double xMiddle,
      double yStart,
      double yMiddle,
      double yMax) {
    tunableXStart = new LoggedTunableNumber(keyPrefix + "/xStart", xStart);
    tunableXMiddle = new LoggedTunableNumber(keyPrefix + "/xMiddle", xMiddle);
    tunableYStart = new LoggedTunableNumber(keyPrefix + "/yStart", yStart);
    tunableYMiddle = new LoggedTunableNumber(keyPrefix + "/yMiddle", yMiddle);
    tunableYMax = new LoggedTunableNumber(keyPrefix + "/yMax", yMax);
    set(xStart, xMiddle, yStart, yMiddle, yMax);
  }

  public final void set(
      double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
    this.xStart = MathUtil.clamp(xStart, 0.0, 0.95);
    this.xMiddle = MathUtil.clamp(xMiddle, this.xStart, 1.0);
    this.yStart = MathUtil.clamp(yStart, 0.0, 1.0);
    this.yMiddle = MathUtil.clamp(yMiddle, this.yStart, 1.0);
    this.yMax = MathUtil.clamp(yMax, this.yMiddle, 1.0);
    this.slope1 = (this.xStart < this.xMiddle) ? (this.yMiddle - this.yStart) / (this.xMiddle - this.xStart) : 0.0;
    this.slope2 = (this.xMiddle < 1.0) ? (this.yMax - this.yMiddle) / (1.0 - this.xMiddle) : 0.0;
  }

  public void pollTunables() {
    if (tunableXStart == null) {
      return;
    }
    LoggedTunableNumber.ifChanged(
        hashCode(),
        vals -> set(vals[0], vals[1], vals[2], vals[3], vals[4]),
        tunableXStart,
        tunableXMiddle,
        tunableYStart,
        tunableYMiddle,
        tunableYMax);
  }

  public double transfer(double x) {
    double xAbs = Math.abs(x);
    if (xAbs < xStart) {
      return 0.0;
    }
    double yAbs;
    if (xAbs < xMiddle) {
      yAbs = yStart + slope1 * (xAbs - xStart);
    } else if (xAbs < 1.0) {
      yAbs = yMiddle + slope2 * (xAbs - xMiddle);
    } else {
      yAbs = yMax;
    }
    yAbs = MathUtil.clamp(yAbs, 0.0, yMax);
    return x >= 0.0 ? yAbs : -yAbs;
  }
}
