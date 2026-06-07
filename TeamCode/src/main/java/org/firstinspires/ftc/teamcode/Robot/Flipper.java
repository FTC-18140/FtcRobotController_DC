package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Flipper {

    private Servo flipperServo = null;
    private Telemetry telemetry = null;
    private double currentServoPosition = 0.0;

    // --- Tunable Constants via FTC Dashboard ---
    private static double EXTENDED_POSITION = 0.1;
    private static double RETRACTED_POSITION = 0.015;
    private static boolean TELEM = false;

    // --- State Management ---
    public enum State {EXTENDED, RETRACTED}

    private State currentState = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            flipperServo = hwMap.servo.get("flipper");

        } catch (RuntimeException e) {
            telemetry.addData("Servo \"flipper\" not found", 0);
        }
        retract();
    }

    // --- High-Level Commands ---

    void extend() {
        currentState = State.EXTENDED;
    }

    private void retract() {
        currentState = State.RETRACTED;
    }

    // --- State Checking ---

    boolean isRetracted() {
        return 0.05 > Math.abs(currentServoPosition - RETRACTED_POSITION);
    }

    public void update() {
        currentServoPosition = flipperServo.getPosition();

        // Refactored to have a single exit point
        if (null != currentState) {
            switch (currentState) {
                case EXTENDED:
                    flipperServo.setPosition(EXTENDED_POSITION);
                    break;
                case RETRACTED:
                    flipperServo.setPosition(RETRACTED_POSITION);
                    break;
            }
        }

        if (TELEM) {
            telemetry.addData("Flipper State", currentState);
            telemetry.addData("Flipper Servo Pos", currentServoPosition);
        }
    }
}
