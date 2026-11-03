package frc.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Centralized CAN signal registry — collects all StatusSignals across subsystems and refreshes them
 * in a single {@code BaseStatusSignal.refreshAll()} call per bus, reducing CAN round-trips.
 */
public final class PhoenixSignals {
  private PhoenixSignals() {}

  private static int nextGroupId = 0;
  private static final Map<String, List<BaseStatusSignal>> busBuckets = new LinkedHashMap<>();
  private static final Map<Integer, GroupInfo> groups = new HashMap<>();
  private static final Map<String, BaseStatusSignal[]> cachedArrays = new HashMap<>();
  private static boolean dirty = true;

  private static final class GroupInfo {
    final BaseStatusSignal[] signals;
    boolean lastOk = false;

    GroupInfo(BaseStatusSignal[] signals) {
      this.signals = signals;
    }
  }

  /** Registers signals for batch refresh. Call from IO constructors. */
  public static int register(String busName, BaseStatusSignal... signals) {
    if (busName == null) {
      throw new IllegalArgumentException("busName must not be null");
    }
    if (signals == null || signals.length == 0) {
      throw new IllegalArgumentException("signals must not be null or empty");
    }

    int groupId = nextGroupId++;
    busBuckets.computeIfAbsent(busName, k -> new ArrayList<>()).addAll(Arrays.asList(signals));
    groups.put(groupId, new GroupInfo(signals.clone()));
    dirty = true;
    return groupId;
  }

  /** Refreshes all registered signals — one CAN round-trip per bus. */
  public static void refreshAll() {
    if (groups.isEmpty()) {
      return;
    }

    if (dirty) {
      cachedArrays.clear();
      for (var entry : busBuckets.entrySet()) {
        List<BaseStatusSignal> signals = entry.getValue();
        cachedArrays.put(entry.getKey(), signals.toArray(new BaseStatusSignal[0]));
      }
      dirty = false;
    }

    for (BaseStatusSignal[] busSignals : cachedArrays.values()) {
      BaseStatusSignal.refreshAll(busSignals);
    }

    for (GroupInfo group : groups.values()) {
      boolean ok = true;
      for (BaseStatusSignal signal : group.signals) {
        if (!signal.getStatus().isOK()) {
          ok = false;
          break;
        }
      }
      group.lastOk = ok;
    }
  }

  public static boolean isOK(int groupId) {
    GroupInfo info = groups.get(groupId);
    return info != null && info.lastOk;
  }

  public static int signalCount() {
    int count = 0;
    for (List<BaseStatusSignal> signals : busBuckets.values()) {
      count += signals.size();
    }
    return count;
  }

  public static void reset() {
    nextGroupId = 0;
    busBuckets.clear();
    groups.clear();
    cachedArrays.clear();
    dirty = true;
  }
}
