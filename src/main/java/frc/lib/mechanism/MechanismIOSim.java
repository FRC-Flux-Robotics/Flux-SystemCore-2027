package frc.lib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.fluxfig.FluxFig.MechanismSpec;

/**
 * Physics simulation implementation of {@link MechanismIO} supporting {@link ElevatorSim}
 * (`ELEVATOR_STATIC`), {@link SingleJointedArmSim} (`ARM_COSINE`), and {@link DCMotorSim} (`NONE`)
 * for desktop simulation and Ghost Superstructure mode on bare chassis (`CORAL` / `CHASSIS_3`).
 */
public class MechanismIOSim implements MechanismIO {
  public static final double ELEVATOR_METERS_PER_ROTATION = 0.030;
  private static final double ELEVATOR_DRUM_RADIUS_M =
      ELEVATOR_METERS_PER_ROTATION / (2.0 * Math.PI);
  private static final double ELEVATOR_CARRIAGE_MASS_KG = 4.2;
  private static final double ARM_LENGTH_M = 0.45;
  private static final double ARM_MOI_KGM2 = 0.065;

  public enum SimPlantType {
    DC_MOTOR,
    ELEVATOR_STATIC,
    ARM_COSINE
  }

  private final MechanismSpec spec;
  private final SimPlantType plantType;
  private final DCMotorSim dcSim;
  private final ElevatorSim elevatorSim;
  private final SingleJointedArmSim armSim;
  private final double armRadPerPosRot;

  private double kP;
  private double kI;
  private double kD;
  private double kS;
  private double kV;
  private double kG;

  private double appliedVolts = 0.0;
  private double targetPosRot = 0.0;
  private double targetVelRps = 0.0;
  private boolean closedLoopPos = false;
  private boolean closedLoopVel = false;

  public MechanismIOSim(MechanismSpec spec) {
    this.spec = spec;
    this.kP = spec.kP();
    this.kI = spec.kI();
    this.kD = spec.kD();
    this.kS = spec.kS();
    this.kV = spec.kV();
    this.kG = spec.kG();

    DCMotor kraken = DCMotor.getKrakenX60Foc(1);
    double gearing = Math.max(1.0, spec.gearRatio());

    if ("ELEVATOR_STATIC".equalsIgnoreCase(spec.gravityType())) {
      this.plantType = SimPlantType.ELEVATOR_STATIC;
      double minMeters = Math.max(0.0, spec.minPosRot() * ELEVATOR_METERS_PER_ROTATION);
      double maxMeters = Math.max(minMeters + 0.1, spec.maxPosRot() * ELEVATOR_METERS_PER_ROTATION);
      this.elevatorSim =
          new ElevatorSim(
              kraken,
              gearing,
              ELEVATOR_CARRIAGE_MASS_KG,
              ELEVATOR_DRUM_RADIUS_M,
              minMeters,
              maxMeters,
              true,
              minMeters);
      this.armSim = null;
      this.dcSim = null;
      this.armRadPerPosRot = 2.0 * Math.PI;
    } else if ("ARM_COSINE".equalsIgnoreCase(spec.gravityType())) {
      this.plantType = SimPlantType.ARM_COSINE;
      double armGearing = (spec.gearRatio() > 1.5) ? spec.gearRatio() : 40.0;
      double spanRot = Math.abs(spec.maxPosRot() - spec.minPosRot());
      this.armRadPerPosRot = (spanRot > 2.0) ? (2.0 * Math.PI / armGearing) : (2.0 * Math.PI);
      double minRad = spec.minPosRot() * this.armRadPerPosRot;
      double maxRad = Math.max(minRad + 0.05, spec.maxPosRot() * this.armRadPerPosRot);
      double startRad = MathUtil.clamp(0.0, minRad, maxRad);
      this.armSim =
          new SingleJointedArmSim(
              kraken, armGearing, ARM_MOI_KGM2, ARM_LENGTH_M, minRad, maxRad, true, startRad);
      this.elevatorSim = null;
      this.dcSim = null;
    } else {
      this.plantType = SimPlantType.DC_MOTOR;
      this.dcSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kraken, 0.002, gearing), kraken);
      this.elevatorSim = null;
      this.armSim = null;
      this.armRadPerPosRot = 2.0 * Math.PI;
    }
  }

  public SimPlantType getPlantType() {
    return plantType;
  }

  @Override
  public void setPosition(double positionRotations) {
    targetPosRot = MathUtil.clamp(positionRotations, spec.minPosRot(), spec.maxPosRot());
    closedLoopPos = true;
    closedLoopVel = false;
  }

  @Override
  public void setVelocity(double velocityRPS) {
    targetVelRps = velocityRPS;
    closedLoopVel = true;
    closedLoopPos = false;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopPos = false;
    closedLoopVel = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kG = kG;
  }

  @Override
  public void stop() {
    closedLoopPos = false;
    closedLoopVel = false;
    appliedVolts = 0.0;
  }

  @Override
  public void resetPosition() {
    if (plantType == SimPlantType.ELEVATOR_STATIC) {
      elevatorSim.setState(0.0, 0.0);
    } else if (plantType == SimPlantType.ARM_COSINE) {
      armSim.setState(0.0, 0.0);
    } else {
      dcSim.setState(0.0, 0.0);
    }
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    double posRot = readPositionRotations();
    double velRps = readVelocityRps();

    if (closedLoopPos) {
      double err = targetPosRot - posRot;
      double gravityFf =
          (plantType == SimPlantType.ARM_COSINE)
              ? kG * Math.cos(posRot * armRadPerPosRot)
              : kG;
      double kpScale = (plantType == SimPlantType.ARM_COSINE) ? 25.0 : 1.0;
      double ksTerm = (Math.abs(err) > 1e-3) ? kS * Math.signum(err) : 0.0;
      appliedVolts =
          MathUtil.clamp(err * kP * kpScale - velRps * kD + ksTerm + gravityFf, -12.0, 12.0);
    } else if (closedLoopVel) {
      double err = targetVelRps - velRps;
      appliedVolts =
          MathUtil.clamp(
              kS * Math.signum(targetVelRps) + targetVelRps * kV + err * kP,
              -12.0,
              12.0);
    }

    double rawCurrentA;
    if (plantType == SimPlantType.ELEVATOR_STATIC) {
      elevatorSim.setInputVoltage(appliedVolts);
      elevatorSim.update(0.020);
      rawCurrentA = elevatorSim.getCurrentDrawAmps();
    } else if (plantType == SimPlantType.ARM_COSINE) {
      armSim.setInputVoltage(appliedVolts);
      armSim.update(0.020);
      rawCurrentA = armSim.getCurrentDrawAmps();
    } else {
      dcSim.setInputVoltage(appliedVolts);
      dcSim.update(0.020);
      rawCurrentA = dcSim.getCurrentDrawAmps();
    }

    inputs.positionRotations = readPositionRotations();
    inputs.velocityRPS = readVelocityRps();
    inputs.statorCurrentA = Math.min(spec.statorLimitAmps(), Math.abs(rawCurrentA));
    inputs.supplyCurrentA =
        Math.min(
            spec.supplyLimitAmps(), inputs.statorCurrentA * (Math.abs(appliedVolts) / 12.0));
    inputs.appliedVoltage = appliedVolts;
    inputs.tempCelsius = 25.0;
    inputs.motorConnected = true;
  }

  private double readPositionRotations() {
    if (plantType == SimPlantType.ELEVATOR_STATIC) {
      return elevatorSim.getPositionMeters() / ELEVATOR_METERS_PER_ROTATION;
    } else if (plantType == SimPlantType.ARM_COSINE) {
      return armSim.getAngleRads() / armRadPerPosRot;
    }
    return dcSim.getAngularPositionRotations();
  }

  private double readVelocityRps() {
    if (plantType == SimPlantType.ELEVATOR_STATIC) {
      return elevatorSim.getVelocityMetersPerSecond() / ELEVATOR_METERS_PER_ROTATION;
    } else if (plantType == SimPlantType.ARM_COSINE) {
      return armSim.getVelocityRadPerSec() / armRadPerPosRot;
    }
    return dcSim.getAngularVelocityRPM() / 60.0;
  }
}
