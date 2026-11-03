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
 * AdvantageKit CAN Battery Fuel Gauge ({@code Playing With Fusion BFG SEN-36005}) reader wired to
 * {@code auxCanBus} ({@code "rio"} native 1 Mbps CAN 2.0 on roboRIO / {@code "can_s4"} on
 * SystemCore).
 *
 * <p>Decodes 29-bit FRC extended CAN arbitration frames ({@link #API_ID_SERIAL}, {@link
 * #API_ID_TELEMETRY}, {@link #API_ID_ENERGY_EXTENDED}) from the inline {@code SEN-36005} sensor,
 * automatically resolves the connected battery serial / QR code against {@code batteries.csv} at
 * boot, tracks terminal voltage, current, State of Charge ({@code SoC %}), State of Health ({@code
 * SoH %}), and internal resistance ({@code R_int mOhm}), and stamps the resolved {@code
 * batteryName} into {@code run-meta.json} with zero manual phone or QR steps.
 */
public class BatteryIO {

  /** FRC 5-bit CAN Device Type for Miscellaneous / Power sensors. */
  public static final int BFG_DEVICE_TYPE_ID = 10;

  /** WPILib HAL CAN Manufacturer ID for Playing With Fusion ({@code HAL_CAN_Man_kPlayingWithFusion}). */
  public static final int PWF_MANUFACTURER_ID = 6;

  /** Alternate Playing With Fusion CAN Manufacturer ID used in legacy sensor dictionary docs. */
  public static final int PWF_ALT_MANUFACTURER_ID = 10;

  /** 10-bit FRC CAN API ID carrying the 8-byte ASCII or uint32 hardware serial number. */
  public static final int API_ID_SERIAL = 0x010;

  /**
   * 10-bit FRC CAN API ID carrying 8-byte big-endian primary telemetry: {@code [voltage_mV_u16,
   * current_dA_i16, soc_tenthPct_u16, soh_pct_u8, rInt_tenthMohm_u8]}.
   */
  public static final int API_ID_TELEMETRY = 0x011;

  /**
   * 10-bit FRC CAN API ID carrying 8-byte big-endian energy &amp; peak/trough metrics: {@code
   * [consumed_mAh_u16, minVoltage_mV_u16, peakCurrent_dA_i16, temp_tenthC_i16]}.
   */
  public static final int API_ID_ENERGY_EXTENDED = 0x012;

  /** Default CAN node ID for a single inline battery sensor on the main battery lead. */
  public static final int DEFAULT_BFG_DEVICE_ID = 0;

  /** Loggable inputs for the inline Playing With Fusion Battery Fuel Gauge (`SEN-36005`). */
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
  private double maxAllowedRintMilliOhms = 24.0;
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
          "Playing With Fusion BFG (SEN-36005) is 1 Mbps CAN 2.0 and must be wired to auxCanBus"
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

  /** Packs a 29-bit Playing With Fusion BFG arbitration ID for the given 10-bit {@code apiId}. */
  public static int buildBfgArbitrationId(int apiId, int deviceId) {
    return buildArbitrationId(BFG_DEVICE_TYPE_ID, PWF_MANUFACTURER_ID, apiId, deviceId);
  }

  /** Extracts the 5-bit Device Type (`bits [28:24]`) from a 29-bit FRC CAN arbitration ID. */
  public static int extractDeviceType(int arbitrationId) {
    return (arbitrationId >>> 24) & 0x1F;
  }

  /** Extracts the 8-bit Manufacturer ID (`bits [23:16]`) from a 29-bit FRC CAN arbitration ID. */
  public static int extractManufacturer(int arbitrationId) {
    return (arbitrationId >>> 16) & 0xFF;
  }

  /** Extracts the 10-bit API ID (`bits [15:6]`) from a 29-bit FRC CAN arbitration ID. */
  public static int extractApiId(int arbitrationId) {
    return (arbitrationId >>> 6) & 0x3FF;
  }

  /** Extracts the 6-bit Device ID (`bits [5:0]`) from a 29-bit FRC CAN arbitration ID. */
  public static int extractDeviceId(int arbitrationId) {
    return arbitrationId & 0x3F;
  }

  /**
   * Encodes an 8-byte ASCII serial/code frame for {@link #API_ID_SERIAL} (space-padded or truncated
   * to 8 bytes).
   */
  public static byte[] encodeSerialFrame(String asciiSerial) {
    byte[] out = new byte[8];
    byte[] raw =
        (asciiSerial == null ? "" : asciiSerial.trim()).getBytes(StandardCharsets.US_ASCII);
    int len = Math.min(8, raw.length);
    System.arraycopy(raw, 0, out, 0, len);
    return out;
  }

  /**
   * Encodes an 8-byte big-endian telemetry frame for {@link #API_ID_TELEMETRY}: {@code
   * [voltage_mV_u16, current_dA_i16, soc_tenthPct_u16, soh_pct_u8, rInt_tenthMohm_u8]}.
   */
  public static byte[] encodeTelemetryFrame(
      double voltageVolts,
      double currentAmps,
      double socPct,
      double sohPct,
      double rIntMilliOhms) {
    int vMv = Math.max(0, Math.min(65535, (int) Math.round(voltageVolts * 1000.0)));
    int iDa = Math.max(-32768, Math.min(32767, (int) Math.round(currentAmps * 10.0)));
    int socTenth = Math.max(0, Math.min(1000, (int) Math.round(socPct * 10.0)));
    int sohU8 = Math.max(0, Math.min(100, (int) Math.round(sohPct)));
    int rIntTenth = Math.max(0, Math.min(255, (int) Math.round(rIntMilliOhms * 10.0)));

    return new byte[] {
      (byte) ((vMv >>> 8) & 0xFF),
      (byte) (vMv & 0xFF),
      (byte) ((iDa >>> 8) & 0xFF),
      (byte) (iDa & 0xFF),
      (byte) ((socTenth >>> 8) & 0xFF),
      (byte) (socTenth & 0xFF),
      (byte) (sohU8 & 0xFF),
      (byte) (rIntTenth & 0xFF)
    };
  }

  /**
   * Encodes an 8-byte big-endian extended energy frame for {@link #API_ID_ENERGY_EXTENDED}: {@code
   * [consumed_mAh_u16, minVoltage_mV_u16, peakCurrent_dA_i16, temp_tenthC_i16]}.
   */
  public static byte[] encodeExtendedEnergyFrame(
      double consumedMah,
      double minVoltageVolts,
      double peakCurrentAmps,
      double temperatureCelsius) {
    int mah = Math.max(0, Math.min(65535, (int) Math.round(consumedMah)));
    int minVMv = Math.max(0, Math.min(65535, (int) Math.round(minVoltageVolts * 1000.0)));
    int peakIDa = Math.max(-32768, Math.min(32767, (int) Math.round(peakCurrentAmps * 10.0)));
    int tempTenthC = Math.max(-32768, Math.min(32767, (int) Math.round(temperatureCelsius * 10.0)));

    return new byte[] {
      (byte) ((mah >>> 8) & 0xFF),
      (byte) (mah & 0xFF),
      (byte) ((minVMv >>> 8) & 0xFF),
      (byte) (minVMv & 0xFF),
      (byte) ((peakIDa >>> 8) & 0xFF),
      (byte) (peakIDa & 0xFF),
      (byte) ((tempTenthC >>> 8) & 0xFF),
      (byte) (tempTenthC & 0xFF)
    };
  }

  /**
   * Processes a raw 29-bit FRC CAN frame from {@code auxCanBus}. Returns {@code true} if the frame
   * matched this Playing With Fusion BFG sensor and was decoded.
   */
  public boolean processCanFrame(int arbitrationId, byte[] data, int length) {
    if (data == null || length <= 0 || length > data.length) {
      return false;
    }
    int devType = extractDeviceType(arbitrationId);
    int mfg = extractManufacturer(arbitrationId);
    int devId = extractDeviceId(arbitrationId);
    if (devType != BFG_DEVICE_TYPE_ID
        || (mfg != PWF_MANUFACTURER_ID && mfg != PWF_ALT_MANUFACTURER_ID)
        || devId != deviceId) {
      return false;
    }
    int apiId = extractApiId(arbitrationId);
    return processApiPayload(apiId, data, length);
  }

  /** Decodes an 8-byte Playing With Fusion BFG payload for the given 10-bit {@code apiId}. */
  public boolean processApiPayload(int apiId, byte[] data, int length) {
    if (data == null || length < 4) {
      return false;
    }
    switch (apiId) {
      case API_ID_SERIAL -> {
        String serial = decodeSerialPayload(data, length);
        inputs.connected = true;
        inputs.framesReceived++;
        inputs.rawSerialOrCode = serial;
        resolveBatteryFromFluxFig(serial);
        return true;
      }
      case API_ID_TELEMETRY -> {
        if (length < 8) {
          return false;
        }
        int vMv = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
        int iDa = (short) (((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
        int socTenth = ((data[4] & 0xFF) << 8) | (data[5] & 0xFF);
        int sohPct = data[6] & 0xFF;
        int rIntTenth = data[7] & 0xFF;

        double voltageV = vMv / 1000.0;
        double currentA = iDa / 10.0;
        double socPct = Math.min(100.0, socTenth / 10.0);
        double soh = Math.min(100.0, (double) sohPct);

        inputs.connected = true;
        inputs.framesReceived++;
        inputs.voltageVolts = voltageV;
        inputs.currentAmps = currentA;
        inputs.stateOfChargePct = socPct;
        inputs.stateOfHealthPct = soh;
        if (voltageV < inputs.minVoltageVolts || inputs.framesReceived == 1) {
          inputs.minVoltageVolts = voltageV;
        }
        if (Math.abs(currentA) > inputs.peakCurrentAmps) {
          inputs.peakCurrentAmps = Math.abs(currentA);
        }

        if (rIntTenth > 0) {
          inputs.internalResistanceMilliOhms = rIntTenth / 10.0;
        } else if (!Double.isNaN(prevSampleVoltageV)
            && Math.abs(currentA - prevSampleCurrentA) >= 25.0) {
          double dV = prevSampleVoltageV - voltageV;
          double dI = currentA - prevSampleCurrentA;
          double estMohm = (dV / dI) * 1000.0;
          if (estMohm >= 5.0 && estMohm <= 60.0) {
            inputs.internalResistanceMilliOhms = estMohm;
          }
        }
        prevSampleVoltageV = voltageV;
        prevSampleCurrentA = currentA;
        return true;
      }
      case API_ID_ENERGY_EXTENDED -> {
        if (length < 8) {
          return false;
        }
        int mah = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
        int minVMv = ((data[2] & 0xFF) << 8) | (data[3] & 0xFF);
        int peakIDa = (short) (((data[4] & 0xFF) << 8) | (data[5] & 0xFF));
        int tempTenthC = (short) (((data[6] & 0xFF) << 8) | (data[7] & 0xFF));

        inputs.connected = true;
        inputs.framesReceived++;
        inputs.consumedChargeMah = mah;
        inputs.minVoltageVolts = minVMv / 1000.0;
        inputs.peakCurrentAmps = peakIDa / 10.0;
        inputs.temperatureCelsius = tempTenthC / 10.0;
        return true;
      }
      default -> {
        return false;
      }
    }
  }

  private static String decodeSerialPayload(byte[] data, int length) {
    int end = Math.min(8, length);
    boolean printableAscii = true;
    int nonNullLen = 0;
    for (int i = 0; i < end; i++) {
      int b = data[i] & 0xFF;
      if (b == 0) {
        break;
      }
      if (b < 0x20 || b > 0x7E) {
        printableAscii = false;
        break;
      }
      nonNullLen++;
    }
    if (printableAscii && nonNullLen > 0) {
      return new String(data, 0, nonNullLen, StandardCharsets.US_ASCII).trim();
    }
    long u32 =
        ((long) (data[0] & 0xFF) << 24)
            | ((long) (data[1] & 0xFF) << 16)
            | ((long) (data[2] & 0xFF) << 8)
            | (data[3] & 0xFF);
    return String.format("PWF-%08X", u32);
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
    inputs.framesReceived++;
    resolveBatteryFromFluxFig(serialOrQrCode);
  }

  private void resolveBatteryFromFluxFig(String serialOrQrCode) {
    if (serialOrQrCode != null && !serialOrQrCode.isBlank()) {
      String trimmed = serialOrQrCode.trim();
      String prefixed = "FLUX-BAT-" + trimmed;
      for (BatterySpec spec : fluxFig.getBatteries().values()) {
        if (spec.batteryName().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(trimmed)
            || spec.qrCode().equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = spec.batteryName();
          maxAllowedRintMilliOhms = spec.maxAllowedRintMilliOhms();
          return;
        }
      }
      for (Map<String, String> row : fluxFig.getTable("batteries")) {
        String name = row.getOrDefault("batteryName", "");
        String qr = row.getOrDefault("qrCode", "");
        if (name.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(trimmed)
            || qr.equalsIgnoreCase(prefixed)) {
          resolvedBatteryName = name;
          maxAllowedRintMilliOhms =
              Double.parseDouble(row.getOrDefault("maxAllowedRintMilliOhms", "24.0"));
          return;
        }
      }
    }
    resolvedBatteryName =
        (serialOrQrCode == null || serialOrQrCode.isBlank())
            ? "UNKNOWN_BATTERY"
            : serialOrQrCode.trim();
  }

  /** Polls WPILib's hardware CAN stream using a single pre-allocated {@link CANData} buffer. */
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
    if (wpiCanHandle.readPacketNew(API_ID_SERIAL, canData)) {
      processApiPayload(API_ID_SERIAL, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_TELEMETRY, canData)) {
      processApiPayload(API_ID_TELEMETRY, canData.data, canData.length);
    }
    if (wpiCanHandle.readPacketNew(API_ID_ENERGY_EXTENDED, canData)) {
      processApiPayload(API_ID_ENERGY_EXTENDED, canData.data, canData.length);
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

  public int getFramesReceived() {
    return inputs.framesReceived;
  }
}
