package frc.robot;

/**
 * Fleet robot identities for FRC Team 10413 (FLUX Robotics), supporting both SystemCore and
 * roboRIO 2.0 fallback via FluxFig.
 */
public enum RobotIdentity {
  COMP_2027,
  PRACTICE_2027,
  FUEL,
  CORAL,
  CHASSIS_3;

  public static RobotIdentity fromSerialNumber(String serial) {
    return fromSerialOrComments(serial, "");
  }

  public static RobotIdentity fromSerialOrComments(String serial, String comments) {
    String combined =
        ((serial == null ? "" : serial) + " " + (comments == null ? "" : comments))
            .trim()
            .toUpperCase();
    if (combined.isBlank()) {
      return COMP_2027;
    }
    if (combined.contains("CORAL")) {
      return CORAL;
    }
    if (combined.contains("FUEL")) {
      return FUEL;
    }
    if (combined.contains("CHASSIS")) {
      return CHASSIS_3;
    }
    if (combined.contains("PRACTICE")) {
      return PRACTICE_2027;
    }
    return COMP_2027;
  }
}
