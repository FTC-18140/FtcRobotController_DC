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
    public static double EXTENDED_POSITION = 0.45;
    public static double RETRACTED_POSITION = 0.035;

    // --- State Management ---
    public enum State { EXTENDED, RETRACTED }
    private State currentState;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        try {
            flipperServo = hwMap.servo.get("flipper");
        } catch (Exception e) {
            telemetry.addData("Servo \"flipper\" not found", 0);
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

        telemetry.addData("Flipper State", currentState);
        telemetry.addData("Flipper Servo Pos", currentServoPosition);
    }
}
