package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config // Make this class tunable
public class Flipper {

    private Servo flipperServo;
    private Telemetry telemetry;
    private double currentServoPosition;

    // --- Tunable Constants via FTC Dashboard ---
    public static double EXTENDED_POSITION = 0.01;
    public static double RETRACTED_POSITION = 0.1;
    public static boolean TELEM = false;

    // --- State Management ---
    public enum State { EXTENDED, RETRACTED }
    private State currentState;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        try {
            flipperServo = hwMap.servo.get("flipper");
//            flipperServo.setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            addTelemetry("Servo \"flipper\" not found", 0);
        }
        retract();
    }

    // --- High-Level Commands ---

    public void extend() {
        currentState = State.EXTENDED;
    }

    public void retract() {
        currentState = State.RETRACTED;
    }

    // --- State Checking ---

    public boolean isRetracted() {
        return Math.abs(currentServoPosition - RETRACTED_POSITION) < 0.05;
    }

    public void update() {
        currentServoPosition = flipperServo.getPosition();

        // Refactored to have a single exit point
        if (currentState != null) {
            switch (currentState) {
                case EXTENDED:
                    flipperServo.setPosition(EXTENDED_POSITION);
                    break;
                case RETRACTED:
                    flipperServo.setPosition(RETRACTED_POSITION);
                    break;
            }
        }

        addTelemetry("Flipper State", currentState);
        addTelemetry("Flipper Servo Pos", currentServoPosition);
    }

    /**
     * Generic method for Objects
     */
    public void addTelemetry(String name, Object value) {
        if (TELEM) {
            // [TAG   ] (6 chars) + Name (15 chars)
            // %-6.6s  -> Exactly 6 chars, Left Aligned
            // %-15.15s -> Exactly 10 chars, Left Aligned
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());
            String label = String.format("%-10.10s", name);

            telemetry.addData(tag + " " + label, value);
        }
    }

    /**
     * Overloaded method for Doubles (fixed precision + fixed label width)
     */
    public void addTelemetry(String name, double value) {
        if (TELEM) {
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());
            String label = String.format("%-10.10s", name);

            // %10.4f ensures the number itself doesn't jitter
            telemetry.addData(tag + " " + label, String.format("%10.4f", value));
        }
    }

    public void addTelemetry(String line)
    {
        if (TELEM) {
            telemetry.addLine(line);
        }
    }
}
