package frc.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Solves unknown camera mount extrinsics (yaw & X/Y lever arm) and wall AprilTag poses from
 * pure L-slide translations and in-place pivot pairs (AX = XB Hand-Eye Calibration).
 */
public final class ZeroPriorBootstrapSolver {
  public record SolvedCameraExtrinsics(
      double xMeters, double yMeters, double zMeters, double yawRad) {}

  public record PivotObservationPair(
      Rotation2d deltaHeading, Translation2d tagInCamBefore, Translation2d tagInCamAfter) {}

  public record TranslationObservationPair(
      Translation2d robotDeltaField, Translation2d tagDeltaInCam) {}

  private ZeroPriorBootstrapSolver() {}

  /**
   * Phase 1: Pure translation (deltaTheta = 0) cancels camera lever arm and solves camera yaw:
   * deltaP_robot = -R_yaw * deltaP_cam.
   */
  public static double solveCameraYawRad(List<TranslationObservationPair> pairs) {
    double sumSin = 0.0;
    double sumCos = 0.0;
    for (TranslationObservationPair p : pairs) {
      Translation2d negCam = p.tagDeltaInCam().times(-1.0);
      double weight = p.robotDeltaField().getNorm() * negCam.getNorm();
      if (weight < 1e-6) {
        continue;
      }
      double angleRobot = Math.atan2(p.robotDeltaField().getY(), p.robotDeltaField().getX());
      double angleCam = Math.atan2(negCam.getY(), negCam.getX());
      double diff = angleRobot - angleCam;
      sumSin += weight * Math.sin(diff);
      sumCos += weight * Math.cos(diff);
    }
    if (Math.hypot(sumSin, sumCos) < 1e-9) {
      return 0.0;
    }
    return Math.atan2(sumSin, sumCos);
  }

  /**
   * Phase 2: Pure in-place pivot solves camera lever arm (x_c, y_c) in robot frame:
   * (I - R(dTheta)) * c_robot = R_cam * p_tag_0 - R(dTheta) * R_cam * p_tag_1.
   */
  public static SolvedCameraExtrinsics solveExtrinsics(
      List<TranslationObservationPair> translationPairs,
      List<PivotObservationPair> pivotPairs,
      double floorTagHeightCamZ) {
    double yawRad = solveCameraYawRad(translationPairs);
    Rotation2d camRot = new Rotation2d(yawRad);

    double a00 = 0.0;
    double a01 = 0.0;
    double a10 = 0.0;
    double a11 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;

    for (PivotObservationPair pair : pivotPairs) {
      double c = pair.deltaHeading().getCos();
      double s = pair.deltaHeading().getSin();
      double m00 = 1.0 - c;
      double m01 = s;
      double m10 = -s;
      double m11 = 1.0 - c;

      Translation2d p0 = pair.tagInCamBefore().rotateBy(camRot);
      Translation2d p1 = pair.tagInCamAfter().rotateBy(camRot).rotateBy(pair.deltaHeading());
      double rhsX = p1.getX() - p0.getX();
      double rhsY = p1.getY() - p0.getY();

      a00 += m00 * m00 + m10 * m10;
      a01 += m00 * m01 + m10 * m11;
      a10 += m01 * m00 + m11 * m10;
      a11 += m01 * m01 + m11 * m11;
      b0 += m00 * rhsX + m10 * rhsY;
      b1 += m01 * rhsX + m11 * rhsY;
    }

    double det = a00 * a11 - a01 * a10;
    if (Math.abs(det) < 1e-9) {
      return new SolvedCameraExtrinsics(0.0, 0.0, floorTagHeightCamZ, yawRad);
    }
    double xc = (a11 * b0 - a01 * b1) / det;
    double yc = (-a10 * b0 + a00 * b1) / det;
    return new SolvedCameraExtrinsics(xc, yc, floorTagHeightCamZ, yawRad);
  }
}
