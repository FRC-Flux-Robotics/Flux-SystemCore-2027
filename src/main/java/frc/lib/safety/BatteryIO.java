package frc.lib.safety;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxFig.BatterySpec;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * AdvantageKit CAN Battery Fuel Gauge ({@code Playing With Fusion ROB-70001}) reader wired to {@code
 * auxCanBus} ({@code "rio"} native 1 Mbps CAN 2.0 on roboRIO / {@code "can_s4"} on SystemCore).
 *
 * <p>Decodes little-endian 29-bit FRC extended CAN arbitration frames for Manufacturer ID {@code 11
 * (0x0B)} ({@link #API_ID_POWER}, {@link #API_ID_BATTERY_HEALTH}, {@link #API_ID_STATE_OF_CHARGE},
 * {@link #API_ID_NICKNAME_1}, {@link #API_ID_NICKNAME_2}) from {@code ROB-70001}, resolves the
 * battery nickname against {@code batteries.csv}, estimates dynamic internal resistance ({@code
 * R_int}) across {@code >= 25 A} load transients, and stamps {@code batteryName} into {@code
 * run-meta.json}.
 */
public class BatteryIO {

  /** FRC 5-bit CAN Device Type for Miscellaneous / Power sensors ({@code 0x0A}). */
  public static final int BFG_DEVICE_TYPE_ID = 10;

  /** WPILib HAL CAN Manufacturer ID for Playing With Fusion ({@code 0x0B}). */
  public static final int PWF_MANUFACTURER_ID = 11;

  /**
   * 10-bit FRC CAN API ID for PWF {@code Power} ({@code 0x0A0B0000}): LE {@code [0..3] i32
   * current_mA, [4..5] u16 voltage_mV, [6..7] i16 dod_mAh}.
   */
  public static final int API_ID_POWER = 0x000;

  /**
   * 10-bit FRC CAN API ID for PWF {@code BatteryHealth} ({@code 0x0A0B0100}): LE {@code [0..3] u32
   * battAge_s, [4..5] u16 capacity_mAh, [6..7] u16 numCycles}.
   */
  public static final int API_ID_BATTERY_HEALTH = 0x004;

  /**
   * 10-bit FRC CAN API ID for PWF {@code StateOfCharge} ({@code 0x0A0B0700}): LE {@code [0..1] i16
   * soc_mAh, [2..3] i16 effectiveCap_mAh, [4] u8 chargeState, [5..7] i24 dod_Ws}.
   */
  public static final int API_ID_STATE_OF_CHARGE = 0x01C;

  /** 10-bit FRC CAN API ID for PWF {@code Nickname1} ({@code 0x0A0B0F00}): ASCII chars [0..7]. */
  public static final int API_ID_NICKNAME_1 = 0x03C;

  /** 10-bit FRC CAN API ID for PWF {@code Nickname2} ({@code 0x0A0B1000}): ASCII chars [8..10]. */
  public static final int API_ID_NICKNAME_2 = 0x040;

  /** Nominal FRC 12V SLA battery capacity (mAh) used when deriving SoH % from capacity_mAh. */
  public static final double DEFAULT_NOMINAL_CAPACITY_MAH = 18000.0;

  /** Default CAN node ID for a single inline battery sensor on the main battery lead. */
  public static final int DEFAULT_BFG_DEVICE_ID = 0;

  /** Loggable inputs for the inline Playing With Fusion {@code ROB-70001} Battery Fuel Gauge. */
  public static class BatteryIOInputs implements LoggableInputs {
    public boolean connected = false;
    public String rawSerialOrCode = "";
    public double voltageVolts = 12.6;
    public double currentAmps = 0.0;
    public double stateOfChargePct = 100.0;
    public double stateOfHealthPct = 100.0;
    public double internalResistanceMilliOhms = 16.0;
    public double consumedChargeMah = 0.0;
    public double minVoltageVolts = 12.6;
    public double peakCurrentAmps = 0.0;
    public double temperatureCelsius = 25.0;
    public double dodWattHours = 0.0;
    public long batteryAgeSeconds = 0L;
    public int cycleCount = 0;
    public int chargeState = 0;
    public int framesReceived = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("RawSerialOrCode", rawSerialOrCode);
      table.put("VoltageVolts", voltageVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("StateOfChargePct", stateOfChargePct);
      table.put("StateOfHealthPct", stateOfHealthPct);
      table.put("InternalResistanceMilliOhms", internalResistanceMilliOhms);
      table.put("ConsumedChargeMah", consumedChargeMah);
      table.put("MinVoltageVolts", minVoltageVolts);
      table.put("PeakCurrentAmps", peakCurrentAmps);
      table.put("TemperatureCelsius", temperatureCelsius);
      table.put("DodWattHours", dodWattHours);
      table.put("BatteryAgeSeconds", batteryAgeSeconds);
      table.put("CycleCount", cycleCount);
      table.put("ChargeState", chargeState);
      table.put("FramesReceived", framesReceived);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      rawSerialOrCode = table.get("RawSerialOrCode", rawSerialOrCode);
      voltageVolts = table.get("VoltageVolts", voltageVolts);
      currentAmps = table.get("CurrentAmps", currentAmps);
      stateOfChargePct = table.get("StateOfChargePct", stateOfChargePct);
      stateOfHealthPct = table.get("StateOfHealthPct", stateOfHealthPct);
      internalResistanceMilliOhms =
          table.get("InternalResistanceMilliOhms", internalResistanceMilliOhms);
      consumedChargeMah = table.get("ConsumedChargeMah", consumedChargeMah);
      minVoltageVolts = table.get("MinVoltageVolts", minVoltageVolts);
      peakCurrentAmps = table.get("PeakCurrentAmps", peakCurrentAmps);
      temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
      dodWattHours = table.get("DodWattHours", dodWattHours);
      batteryAgeSeconds = table.get("BatteryAgeSeconds", batteryAgeSeconds);
      cycleCount = table.get("CycleCount", cycleCount);
      chargeState = table.get("ChargeState", chargeState);
      framesReceived = table.get("FramesReceived", framesReceived);
    }
  }

  private final String auxCanBus;
  private final int deviceId;
  private final FluxFig fluxFig;
  private final BatteryIOInputs inputs = new BatteryIOInputs();
  private final CANData canData = new CANData();
  private CAN wpiCanHandle;
  private boolean canInitAttempted = false;

  private String resolvedBatteryName = "UNKNOWN_BATTERY";
  private String nicknamePart1 = "";
  private String nicknamePart2 = "";
  private double maxAllowedRintMilliOhms = 24.0;
  private boolean rIntMeasuredFromStep = false;
  private double prevSampleVoltageV = Double.NaN;
  private double prevSampleCurrentA = Double.NaN;

  public BatteryIO(FluxFig fluxFig, String auxCanBus) {
    this(fluxFig, auxCanBus, DEFAULT_BFG_DEVICE_ID);
  }

  public BatteryIO(FluxFig fluxFig, String auxCanBus, int deviceId) {
    if ("Drivetrain".equalsIgnoreCase(auxCanBus)
        || "Mech".equalsIgnoreCase(auxCanBus)
        || "CANdace".equalsIgnoreCase(auxCanBus)
        || "can_s0".equalsIgnoreCase(auxCanBus)
        || "can_s2".equalsIgnoreCase(auxCanBus)) {
      throw new IllegalArgumentException(
          "Playing With Fusion BFG (ROB-70001) is 1 Mbps CAN 2.0 and must be wired to auxCanBus"
              + " ('rio' on roboRIO or 'can_s4' on SystemCore), never high-rate CAN FD bus: "
              + auxCanBus);
    }
    if (deviceId < 0 || deviceId > 63) {
      throw new IllegalArgumentException(
          "BFG 6-bit FRC CAN deviceId must be in [0, 63], got: " + deviceId);
    }
    this.fluxFig = fluxFig;
    this.auxCanBus = auxCanBus;
    this.deviceId = deviceId;
  }

  /**
   * Packs a 29-bit FRC Extended CAN arbitration ID: {@code [deviceType:5 | manufacturer:8 |
   * apiId:10 | deviceId:6]}.
   */
  public static int buildArbitrationId(
      int deviceType, int manufacturer, int apiId, int deviceId) {
    return ((deviceType & 0x1F) << 24)
        | ((manufacturer & 0xFF) << 16)
        | ((apiId & 0x3FF) << 6)
        | (deviceId & 0x3F);
  }

  /** Packs a 29-bit PWF {@code ROB-70001} arbitration ID for the given 10-bit {@code apiId}. */
  public static int buildBfgArbitrationId(int apiId, int deviceId) {
    return buildArbitrationId(BFG_DEVICE_TYPE_ID, PWF_MANUFACTURER_ID, apiId, deviceId);
  }

  public static int extractDeviceType(int arbitrationId) {
    return (arbitrationId >>> 24) & 0x1F;
  }

  public static int extractManufacturer(int arbitrationId) {
    return (arbitrationId >>> 16) & 0xFF;
  }

  public static int extractApiId(int arbitrationId) {
    return (arbitrationId >>> 6) & 0x3FF;
  }

  public static int extractDeviceId(int arbitrationId) {
    return arbitrationId & 0x3F;
  }

  /** Encodes an 8-byte ASCII fragment (chars 0..7) for {@link #API_ID_NICKNAME_1}. */
  public static byte[] encodeNickname1Frame(String nickname) {
    return encodeAsciiSlice(nickname, 0, 8);
  }

  /** Encodes an 8-byte ASCII tail fragment (chars 8..10) for {@link #API_ID_NICKNAME_2}. */
  public static byte[] encodeNickname2Frame(String nickname) {
    return encodeAsciiSlice(nickname, 8, 3);
  }

  private static byte[] encodeAsciiSlice(String text, int offset, int maxLen) {
    byte[] out = new byte[8];
    byte[] raw = (text == null ? "" : text.trim()).getBytes(StandardCharsets.US_ASCII);
    if (offset < raw.length) {
      int copyLen = Math.min(maxLen, raw.length - offset);
      System.arraycopy(raw, offset, out, 0, copyLen);
    }
    return out;
  }

  /**
   * Encodes an 8-byte little-endian PWF {@code Power} frame ({@link #API_ID_POWER}): {@code [0..3]
   * i32 current_mA, [4..5] u16 voltage_mV, [6..7] i16 dod_mAh}.
   */
  public static byte[] encodePowerFrame(
      double currentAmps, double voltageVolts, double dodMah) {
    int currentMa = (int) Math.round(currentAmps * 1000.0);
    int voltageMv = Math.max(0, Math.min(65535, (int) Math.round(voltageVolts * 1000.0)));
    int dodSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(dodMah)));
    return new byte[] {
      (byte) (currentMa & 0xFF),
      (byte) ((currentMa >>> 8) & 0xFF),
      (byte) ((currentMa >>> 16) & 0xFF),
      (byte) ((currentMa >>> 24) & 0xFF),
      (byte) (voltageMv & 0xFF),
      (byte) ((voltageMv >>> 8) & 0xFF),
      (byte) (dodSignedMah & 0xFF),
      (byte) ((dodSignedMah >>> 8) & 0xFF)
    };
  }

  /**
   * Encodes an 8-byte little-endian PWF {@code BatteryHealth} frame ({@link
   * #API_ID_BATTERY_HEALTH}): {@code [0..3] u32 battAge_s, [4..5] u16 capacity_mAh, [6..7] u16
   * numCycles}.
   */
  public static byte[] encodeBatteryHealthFrame(
      long batteryAgeSeconds, double capacityMah, int numCycles) {
    long ageSec = Math.max(0L, batteryAgeSeconds);
    int capMah = Math.max(0, Math.min(65535, (int) Math.round(capacityMah)));
    int cycles = Math.max(0, Math.min(65535, numCycles));
    return new byte[] {
      (byte) (ageSec & 0xFF),
      (byte) ((ageSec >>> 8) & 0xFF),
      (byte) ((ageSec >>> 16) & 0xFF),
      (byte) ((ageSec >>> 24) & 0xFF),
      (byte) (capMah & 0xFF),
      (byte) ((capMah >>> 8) & 0xFF),
      (byte) (cycles & 0xFF),
      (byte) ((cycles >>> 8) & 0xFF)
    };
  }

  /**
   * Encodes an 8-byte little-endian PWF {@code StateOfCharge} frame ({@link
   * #API_ID_STATE_OF_CHARGE}): {@code [0..1] i16 soc_mAh, [2..3] i16 effectiveCap_mAh, [4] u8
   * chargeState, [5..7] i24 dod_Ws}.
   */
  public static byte[] encodeStateOfChargeFrame(
      double socMah, double effectiveCapMah, int chargeState, double dodWattHours) {
    int socSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(socMah)));
    int effCapSignedMah = Math.max(-32768, Math.min(32767, (int) Math.round(effectiveCapMah)));
    int stateU8 = chargeState & 0xFF;
    int dodWs = Math.max(-8_388_608, Math.min(8_388_607, (int) Math.round(dodWattHours * 3600.0)));
    return new byte[] {
      (byte) (socSignedMah & 0xFF),
      (byte) ((socSignedMah >>> 8) & 0xFF),
      (byte) (effCapSignedMah & 0xFF),
      (byte) ((effCapSignedMah >>> 8) & 0xFF),
      (byte) stateU8,
      (byte) (dodWs & 0xFF),
      (byte) ((dodWs >>> 8) & 0xFF),
      (byte) ((dodWs >>> 16) & 0xFF)
    };
  }

  /**
   * Processes a raw 29-bit FRC CAN frame from {@code auxCanBus}. Returns {@code true} if the frame
   * matched this Playing With Fusion {@code ROB-70001} sensor (Mfg ID {@code 11}) and was decoded.
   */
  public boolean processCanFrame(int arbitrationId, byte[] data, int length) {
    if (data == null || length <= 0 || length > data.length) {
      return false;
    }
    int devType = extractDeviceType(arbitrationId);
    int mfg = extractManufacturer(arbitrationId);
    int devId = extractDeviceId(arbitrationId);
    if (devType != BFG_DEVICE_TYPE_ID || mfg != PWF_MANUFACTURER_ID || devId != deviceId) {
      return false;
    }
    return processApiPayload(extractApiId(arbitrationId), data, length);
  }

  /** Decodes a Playing With Fusion {@code ROB-70001} payload for the given 10-bit {@code apiId}. */
  public boolean processApiPayload(int apiId, byte[] data, int length) {
    if (data == null || length <= 0 || length > data.length) {
      return false;
    }
    switch (apiId) {
      case API_ID_POWER -> {
        if (length < 8) {
          return false;
        }
        int currentMa = readLeI32(data, 0);
        int voltageMv = readLeU16(data, 4);
        int dodMah = readLeI16(data, 6);

        double voltageV = voltageMv / 1000.0;
        double currentA = currentMa / 1000.0;

        inputs.connected = true;
        inputs.framesReceived++;
        inputs.voltageVolts = voltageV;
        inputs.currentAmps = currentA;
        inputs.consumedChargeMah = dodMah;
        if (voltageV < inputs.minVoltageVolts || inputs.framesReceived == 1) {
          inputs.minVoltageVolts = voltageV;
        }
        if (Math.abs(currentA) > inputs.peakCurrentAmps) {
          inputs.peakCurrentAmps = Math.abs(currentA);
        }

        if (!Double.isNaN(prevSampleVoltageV)
            && Math.abs(currentA - prevSampleCurrentA) >= 25.0) {
          double dV = prevSampleVoltageV - voltageV;
          double dI = currentA - prevSampleCurrentA;
          double estMohm = (dV / dI) * 1000.0;
          if (estMohm >= 5.0 && estMohm <= 60.0) {
            inputs.internalResistanceMilliOhms = estMohm;
            rIntMeasuredFromStep = true;
          }
        }
        prevSampleVoltageV = voltageV;
        prevSampleCurrentA = currentA;
        return true;
      }
      case API_ID_BATTERY_HEALTH -> {
        if (length < 8) {
          return false;
        }
        long battAgeS = readLeU32(data, 0);
        int capacityMah = readLeU16(data, 4);
        int numCycles = readLeU16(data, 6);

        inputs.connected = true;
        inputs.framesReceived++;
        inputs.batteryAgeSeconds = battAgeS;
        inputs.cycleCount = numCycles;
        inputs.stateOfHealthPct =
            Math.max(0.0, Math.min(100.0, (100.0 * capacityMah) / DEFAULT_NOMINAL_CAPACITY_MAH));
        return true;
      }
      case API_ID_STATE_OF_CHARGE -> {
        if (length < 8) {
          return false;
        }
        int socMah = readLeI16(data, 0);
        int effCapMah = readLeI16(data, 2);
        int chargeState = data[4] & 0xFF;
        int dodWs = readLeI24(data, 5);

        inputs.connected = true;
        inputs.framesReceived++;
        inputs.chargeState = chargeState;
        inputs.dodWattHours = dodWs / 3600.0;
        if (effCapMah > 0) {
          inputs.stateOfChargePct =
              Math.max(0.0, Math.min(100.0, (100.0 * socMah) / effCapMah));
        }
        return true;
      }
      case API_ID_NICKNAME_1 -> {
        nicknamePart1 = decodeAsciiFragment(data, Math.min(8, length));
        updateNicknameResolution();
        return true;
      }
      case API_ID_NICKNAME_2 -> {
        nicknamePart2 = decodeAsciiFragment(data, Math.min(3, length));
        updateNicknameResolution();
        return true;
      }
      default -> {
        return false;
      }
    }
  }

  private void updateNicknameResolution() {
    String combined = (nicknamePart1 + nicknamePart2).trim();
    inputs.connected = true;
    inputs.framesReceived++;
    if (!combined.equals(inputs.rawSerialOrCode)) {
      inputs.rawSerialOrCode = combined;
      resolveBatteryFromFluxFig(combined);
    }
  }

  private static String decodeAsciiFragment(byte[] data, int maxLen) {
    int nonNullLen = 0;
    for (int i = 0; i < maxLen; i++) {
      int b = data[i] & 0xFF;
      if (b == 0) {
        break;
      }
      if (b >= 0x20 && b <= 0x7E) {
        nonNullLen++;
      } else {
        break;
      }
    }
    return (nonNullLen > 0)
        ? new String(data, 0, nonNullLen, StandardCharsets.US_ASCII)
        : "";
  }

  private static int readLeU16(byte[] data, int off) {
    return (data[off] & 0xFF) | ((data[off + 1] & 0xFF) << 8);
  }

  private static int readLeI16(byte[] data, int off) {
    return (short) readLeU16(data, off);
  }

  private static int readLeI24(byte[] data, int off) {
    int raw =
        (data[off] & 0xFF) | ((data[off + 1] & 0xFF) << 8) | ((data[off + 2] & 0xFF) << 16);
    if ((raw & 0x800000) != 0) {
      raw |= 0xFF000000;
    }
    return raw;
  }

  private static int readLeI32(byte[] data, int off) {
    return (data[off] & 0xFF)
        | ((data[off + 1] & 0xFF) << 8)
        | ((data[off + 2] & 0xFF) << 16)
        | ((data[off + 3] & 0xFF) << 24);
  }

  private static long readLeU32(byte[] data, int off) {
    return readLeI32(data, off) & 0xFFFF_FFFFL;
  }

  /** Injects or updates a CAN Fuel Gauge reading (from CAN frame or test harness). */
  public void simulateCanReading(
      String serialOrQrCode,
      double voltageVolts,
      double socPct,
      double sohPct,
      double rIntMilliOhms) {
    inputs.connected = true;
    inputs.rawSerialOrCode = serialOrQrCode;
    inputs.voltageVolts = voltageVolts;
    inputs.stateOfChargePct = socPct;
    inputs.stateOfHealthPct = sohPct;
    inputs.internalResistanceMilliOhms = rIntMilliOhms;
    rIntMeasuredFromStep = true;
    inputs.framesReceived++;
    resolveBatteryFromFluxFig(serialOrQrCode);
  }

  private void resolveBatteryFromFluxFig(String serialOrQrCode) {
    if (serialOrQrCode != null && !serialOrQrCode.isBlank()) {
      String trimmed = serialOrQrCode.trim();
      String prefixed = "FLUX-BAT-" + trimmed;
      for (Map<String, String> row : fluxFig.getTable("batteries")) {
        String name = row.getOrDefault("batteryName", "");
        String qr = row.getOrDefault("qrCode", "");
        if (name.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = name;
          maxAllowedRintMilliOhms =
              Double.parseDouble(row.getOrDefault("maxAllowedRintMilliOhms", "24.0"));
          if (!rIntMeasuredFromStep) {
            inputs.internalResistanceMilliOhms =
                Double.parseDouble(row.getOrDefault("nominalRintMilliOhms", "16.0"));
          }
          return;
        }
      }
      for (BatterySpec spec : fluxFig.getBatteries().values()) {
        if (spec.batteryName().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = spec.batteryName();
          maxAllowedRintMilliOhms = spec.maxAllowedRintMilliOhms();
          return;
        }
      }
    }
    resolvedBatteryName =
        (serialOrQrCode == null || serialOrQrCode.isBlank())
            ? "UNKNOWN_BATTERY"
            : serialOrQrCode.trim();
  }

  /**
   * Polls WPILib's hardware CAN stream using a single pre-allocated {@link CANData} buffer. Note:
   * when cutting over to WPILib 2027, replace 3-arg {@code CAN} / {@code CANData} with {@code
   * CAN(CANPort, ...)} / {@code CANReceiveMessage}.
   */
  private void pollHardwareCan() {
    if (!RobotBase.isReal()) {
      return;
    }
    if (!canInitAttempted) {
      canInitAttempted = true;
      try {
        wpiCanHandle = new CAN(deviceId, PWF_MANUFACTURER_ID, BFG_DEVICE_TYPE_ID);
      } catch (Throwable ignored) {
        wpiCanHandle = null;
      }
    }
    if (wpiCanHandle == null) {
      return;
    }
    if (wpiCanHandle.readPacketNew(API_ID_NICKNAME_1, canData)) {
      processApiPayload(API_ID_NICKNAME_1, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_NICKNAME_2, canData)) {
      processApiPayload(API_ID_NICKNAME_2, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_POWER, canData)) {
      processApiPayload(API_ID_POWER, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_BATTERY_HEALTH, canData)) {
      processApiPayload(API_ID_BATTERY_HEALTH, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_STATE_OF_CHARGE, canData)) {
      processApiPayload(API_ID_STATE_OF_CHARGE, canData.data, canData.length);
    }
  }

  /** Periodic telemetry update logging battery identity, SoC %, SoH %, and health status. */
  public void periodic() {
    pollHardwareCan();
    Logger.processInputs("Battery", inputs);
    Logger.recordOutput("Battery/AuxCanBus", auxCanBus);
    Logger.recordOutput("Battery/DeviceId", deviceId);
    Logger.recordOutput("Battery/Name", resolvedBatteryName);
    Logger.recordOutput("Battery/Healthy", isHealthy());
  }

  public boolean isHealthy() {
    return inputs.connected
        && inputs.stateOfChargePct >= 75.0
        && inputs.internalResistanceMilliOhms <= maxAllowedRintMilliOhms;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getResolvedBatteryName() {
    return resolvedBatteryName;
  }

  public String getAuxCanBus() {
    return auxCanBus;
  }

  public int getDeviceId() {
    return deviceId;
  }

  public double getVoltageVolts() {
    return inputs.voltageVolts;
  }

  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  public double getStateOfChargePct() {
    return inputs.stateOfChargePct;
  }

  public double getStateOfHealthPct() {
    return inputs.stateOfHealthPct;
  }

  public double getInternalResistanceMilliOhms() {
    return inputs.internalResistanceMilliOhms;
  }

  public double getConsumedChargeMah() {
    return inputs.consumedChargeMah;
  }

  public double getMinVoltageVolts() {
    return inputs.minVoltageVolts;
  }

  public double getPeakCurrentAmps() {
    return inputs.peakCurrentAmps;
  }

  public int getCycleCount() {
    return inputs.cycleCount;
  }

  public long getBatteryAgeSeconds() {
    return inputs.batteryAgeSeconds;
  }

  public int getFramesReceived() {
    return inputs.framesReceived;
  }
}
