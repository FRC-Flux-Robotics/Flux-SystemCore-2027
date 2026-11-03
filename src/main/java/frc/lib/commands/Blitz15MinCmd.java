package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.littletonrobotics.junction.Logger;

/**
 * Stateful, Checkpoint-Resumable 15-Minute Floor Blitz Sequence ({@code Blitz15MinCmd}).
 *
 * <p>Executes 4 independent stages in order:
 *
 * <ol>
 *   <li>{@code STAGE_1_PIT_CHECK} ({@link PitSystemCheckCmd})
 *   <li>{@code STAGE_2_HOMING} ({@link HardStopHomingCmd})
 *   <li>{@code STAGE_3_BOOTSTRAP} ({@link ZeroPriorBootstrapCmd})
 *   <li>{@code STAGE_4_STRESS_SUITE} ({@link ShopStressSuiteCmd})
 * </ol>
 *
 * <p>Persists completed stage indices to {@code /tmp/blitz-progress.json} (or a custom test path)
 * so that if a battery is swapped, a breaker trips, or the {@code LT} Dead-Man trigger is released
 * mid-blitz, re-running {@code Blitz15MinCmd} resumes directly at the unfinished stage instead of
 * restarting from {@code 00:00}.
 */
public class Blitz15MinCmd extends Command {
  public static final Path DEFAULT_CHECKPOINT_PATH = Path.of("/tmp/blitz-progress.json");

  private final Command[] stages;
  private final String[] stageNames = {
    "STAGE_1_PIT_CHECK", "STAGE_2_HOMING", "STAGE_3_BOOTSTRAP", "STAGE_4_STRESS_SUITE"
  };
  private final Path checkpointPath;

  private int currentStageIndex = 0;
  private boolean activeStageInitialized = false;

  public Blitz15MinCmd(
      PitSystemCheckCmd pitCheck,
      HardStopHomingCmd homing,
      ZeroPriorBootstrapCmd bootstrap,
      ShopStressSuiteCmd stressSuite) {
    this(pitCheck, homing, bootstrap, stressSuite, DEFAULT_CHECKPOINT_PATH);
  }

  public Blitz15MinCmd(
      PitSystemCheckCmd pitCheck,
      HardStopHomingCmd homing,
      ZeroPriorBootstrapCmd bootstrap,
      ShopStressSuiteCmd stressSuite,
      Path checkpointPath) {
    this.stages = new Command[] {pitCheck, homing, bootstrap, stressSuite};
    this.checkpointPath = checkpointPath;
    for (Command stage : this.stages) {
      addRequirements(stage.getRequirements().toArray(new Subsystem[0]));
    }
  }

  @Override
  public void initialize() {
    currentStageIndex = readCheckpointStageIndex();
    activeStageInitialized = false;
    Logger.recordOutput("Blitz15Min/ResumedAtStageIndex", currentStageIndex);
  }

  @Override
  public void execute() {
    if (currentStageIndex >= stages.length) {
      return;
    }

    Command active = stages[currentStageIndex];
    if (!activeStageInitialized) {
      active.initialize();
      activeStageInitialized = true;
      Logger.recordOutput("Blitz15Min/ActiveStage", stageNames[currentStageIndex]);
    }

    active.execute();
    if (active.isFinished()) {
      active.end(false);
      currentStageIndex++;
      activeStageInitialized = false;
      writeCheckpointStageIndex(currentStageIndex);
    }
  }

  @Override
  public boolean isFinished() {
    return currentStageIndex >= stages.length;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted
        && currentStageIndex < stages.length
        && activeStageInitialized) {
      stages[currentStageIndex].end(true);
    }
    Logger.recordOutput("Blitz15Min/Completed", !interrupted && isFinished());
  }

  public void resetCheckpoint() {
    currentStageIndex = 0;
    writeCheckpointStageIndex(0);
  }

  public int getCurrentStageIndex() {
    return currentStageIndex;
  }

  private int readCheckpointStageIndex() {
    if (checkpointPath == null || !Files.exists(checkpointPath)) {
      return 0;
    }
    try {
      String content = Files.readString(checkpointPath, StandardCharsets.UTF_8);
      int idx = content.indexOf("\"completedStages\":");
      if (idx >= 0) {
        String tail = content.substring(idx + "\"completedStages\":".length()).trim();
        int numEnd = 0;
        while (numEnd < tail.length() && Character.isDigit(tail.charAt(numEnd))) {
          numEnd++;
        }
        if (numEnd > 0) {
          int parsed = Integer.parseInt(tail.substring(0, numEnd));
          return Math.max(0, Math.min(stages.length, parsed));
        }
      }
    } catch (IOException | NumberFormatException ignored) {
    }
    return 0;
  }

  private void writeCheckpointStageIndex(int completedStages) {
    if (checkpointPath == null) {
      return;
    }
    try {
      if (checkpointPath.getParent() != null) {
        Files.createDirectories(checkpointPath.getParent());
      }
      String json = "{\"completedStages\": " + completedStages + "}\n";
      Files.writeString(checkpointPath, json, StandardCharsets.UTF_8);
    } catch (IOException ignored) {
    }
  }
}
