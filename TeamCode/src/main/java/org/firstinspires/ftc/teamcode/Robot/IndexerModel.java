package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.IndexerFacade.BallState;

/**
 * Central model for indexer ball state.
 *
 * <p>This class owns all logical slot-content bookkeeping for the 3-slot indexer:
 * <ul>
 *   <li>Current color/occupancy in each slot</li>
 *   <li>Which slots have already been consumed/fired in the active sequence</li>
 *   <li>Rotation of slot state when the turnstile is commanded to rotate</li>
 *   <li>Sensor-to-slot color mapping rules</li>
 * </ul>
 *
 * <p>{@link IndexerFacade} should use this class as the single source of truth for ball modeling,
 * while handling high-level mechanism orchestration and state transitions.
 */
public class IndexerModel {
    /** Number of indexed slots in the mechanism. */
    private static final int SLOT_COUNT = 3;

    /** Logical slot occupancy/colors. */
    private final BallState[] slots = {BallState.VACANT, BallState.VACANT, BallState.VACANT};
    /** Per-slot "already used/fired" flags for sequence planning. */
    private final boolean[] slotsFired = new boolean[SLOT_COUNT];

    /** Resets all slots to vacant and clears fired flags. */
    public void reset() {
        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = BallState.VACANT;
            slotsFired[i] = false;
        }
    }

    /** @return number of slots represented by this model. */
    public int getSlotCount() {
        return SLOT_COUNT;
    }

    /**
     * Bulk-load slot states.
     *
     * @param states slot array of size {@link #SLOT_COUNT}; ignored when null or wrong size
     */
    public void setSlots(BallState[] states) {
        if (states == null || states.length != SLOT_COUNT) return;
        for (int i = 0; i < SLOT_COUNT; i++) {
            // Copy each entry so caller cannot mutate internal array reference.
            slots[i] = states[i];
        }
    }

    /**
     * Returns slot content.
     *
     * @param slot slot index [0..2]
     * @return current state, or {@link BallState#VACANT} when index is invalid
     */
    public BallState getSlot(int slot) {
        return isValidSlot(slot) ? slots[slot] : BallState.VACANT;
    }

    /** @return cloned snapshot of slot states. */
    public BallState[] copySlots() {
        return slots.clone();
    }

    /** @return cloned snapshot of fired flags. */
    public boolean[] copyFiredSlots() {
        return slotsFired.clone();
    }

    /** @return true when no slot is vacant. */
    public boolean isFull() {
        for (BallState slot : slots) {
            if (slot == BallState.VACANT) return false;
        }
        return true;
    }

    /**
     * Tests whether a target matches the state at a slot.
     *
     * @param target requested target color/state
     * @param slot slot index [0..2]
     * @return true when slot is valid and matches target
     */
    public boolean matches(BallState target, int slot) {
        if (!isValidSlot(slot)) return false;
        return matches(target, slots[slot]);
    }

    /**
     * Tests whether a candidate state matches a target requirement.
     *
     * <p>{@link BallState#ALL} matches any non-vacant slot.
     */
    public boolean matches(BallState target, BallState candidate) {
        return candidate == target || (target == BallState.ALL && candidate != BallState.VACANT);
    }

    /**
     * Marks a slot consumed in the logical model.
     *
     * @param slot slot index [0..2]
     */
    public void markSlotUsed(int slot) {
        if (!isValidSlot(slot)) return;
        // Once consumed by sequence selection, slot is logically vacant until refreshed from sensors.
        slots[slot] = BallState.VACANT;
        slotsFired[slot] = true;
    }

    /**
     * @param slot slot index [0..2]
     * @return true when this slot has been marked as consumed/fired
     */
    public boolean wasSlotFired(int slot) {
        return isValidSlot(slot) && slotsFired[slot];
    }

    /** Clears all fired flags while preserving slot occupancy state. */
    public void clearFiredFlags() {
        for (int i = 0; i < SLOT_COUNT; i++) {
            slotsFired[i] = false;
        }
    }

    /**
     * Rotates model state counter-clockwise by N steps.
     *
     * @param steps number of CCW steps; can be any integer and is normalized mod slot count
     */
    public void rotateCCW(int steps) {
        int normalizedSteps = normalizeSlot(steps);
        for (int i = 0; i < normalizedSteps; i++) {
            // Rotate occupancy and fired flags together to keep them aligned per physical slot.
            rotateArrayCCW(slots);
            rotateArrayCCW(slotsFired);
        }
    }

    /** Rotates the slot state array by one CCW step. */
    private void rotateArrayCCW(BallState[] arr) {
        BallState temp = arr[SLOT_COUNT - 1];
        for (int i = SLOT_COUNT - 1; i > 0; i--) {
            arr[i] = arr[i - 1];
        }
        arr[0] = temp;
    }

    /** Rotates the fired-flag array by one CCW step. */
    private void rotateArrayCCW(boolean[] arr) {
        boolean temp = arr[SLOT_COUNT - 1];
        for (int i = SLOT_COUNT - 1; i > 0; i--) {
            arr[i] = arr[i - 1];
        }
        arr[0] = temp;
    }

    /**
     * Updates logical slots from hardware sensors.
     *
     * <p>Each logical slot is represented by two sensors (A and B):
     * <ul>
     *   <li>PURPLE has highest priority if either sensor reports purple</li>
     *   <li>GREEN when either sensor reports green and no purple is seen</li>
     *   <li>VACANT otherwise</li>
     * </ul>
     */
    public void updateFromSensors(BallSensor[] ballSensors) {
        for (int i = 0; i < SLOT_COUNT; i++) {
            BallSensor sensorA = ballSensors[i * 2];
            BallSensor sensorB = ballSensors[i * 2 + 1];
            BallSensor.BallColor colorA = sensorA.getDetectedColor();
            BallSensor.BallColor colorB = sensorB.getDetectedColor();

            // Resolve pair readings into one logical slot state.
            if (colorA == BallSensor.BallColor.PURPLE || colorB == BallSensor.BallColor.PURPLE) {
                slots[i] = BallState.PURPLE;
            } else if (colorA == BallSensor.BallColor.GREEN || colorB == BallSensor.BallColor.GREEN) {
                slots[i] = BallState.GREEN;
            } else {
                slots[i] = BallState.VACANT;
            }

            // Keep per-sensor debug telemetry available.
            sensorA.addTelemetry();
            sensorB.addTelemetry();
        }
    }

    /** Normalizes any integer into a valid slot offset [0..SLOT_COUNT-1]. */
    private int normalizeSlot(int slot) {
        return (slot % SLOT_COUNT + SLOT_COUNT) % SLOT_COUNT;
    }

    /** @return true if slot index is within bounds. */
    private boolean isValidSlot(int slot) {
        return slot >= 0 && slot < SLOT_COUNT;
    }
}
