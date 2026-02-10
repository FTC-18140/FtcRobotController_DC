package org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Robot.IndexerFacade.BallState.GREEN;
import static org.firstinspires.ftc.teamcode.Robot.IndexerFacade.BallState.PURPLE;
import static org.firstinspires.ftc.teamcode.Robot.IndexerFacade.BallState.VACANT;

import org.firstinspires.ftc.teamcode.Robot.IndexerFacade.BallState;
public class IndexerModel {

    // Logical slots — index 0 = oldest ball, index 2 = newest / next to shoot normally
    private BallState[] logicalSlots = {VACANT, VACANT, VACANT};

    // Which **logical** slot is currently in shooting position
    private int currentShooterLogicalIndex = 2;

    // ───────────────────────────────────────────────────────────────
    // Core operations
    // ───────────────────────────────────────────────────────────────

    public void rotateCCW() {
        currentShooterLogicalIndex = (currentShooterLogicalIndex + 1) % 3;
    }

    public void rotateCW() {
        currentShooterLogicalIndex = (currentShooterLogicalIndex + 2) % 3; // -1 mod 3
    }

    public void shootCurrent() {
        logicalSlots[currentShooterLogicalIndex] = VACANT;
    }

    public void setLogicalSlot(int logicalIndex, BallState color) {
        if (logicalIndex >= 0 && logicalIndex < 3)
            logicalSlots[logicalIndex] = color;
    }

    // ───────────────────────────────────────────────────────────────
    // Very useful queries
    // ───────────────────────────────────────────────────────────────

    public BallState getShooterSlotContent() {
        return logicalSlots[currentShooterLogicalIndex];
    }

    public int findNextLogicalSlotOfColor(BallState wanted, int startFromLogical) {
        for (int i = 0; i < 3; i++) {
            int idx = (startFromLogical + i) % 3;
            if (logicalSlots[idx] == wanted) return idx;
        }
        return -1;
    }

    public int findAnyLogicalSlotOfColor(BallState wanted) {
        return findNextLogicalSlotOfColor(wanted, 0);
    }

    public boolean hasColor(BallState color) {
        return findAnyLogicalSlotOfColor(color) >= 0;
    }

    public int countBalls() {
        int count = 0;
        for (BallState s : logicalSlots)
            if (s == GREEN || s == PURPLE) count++;
        return count;
    }

    public String getDebugState() {
        return String.format("[%s | %s | %s]  ← shooter at logical %d (%s)",
                logicalSlots[0], logicalSlots[1], logicalSlots[2],
                currentShooterLogicalIndex, logicalSlots[currentShooterLogicalIndex]);
    }

    // Optional: reset / set all at once
    public void reset() {
        for (int i = 0; i < 3; i++) logicalSlots[i] = VACANT;
        currentShooterLogicalIndex = 2;
    }
}
