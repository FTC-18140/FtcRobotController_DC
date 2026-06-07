package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Kickstand {
    private Servo kickstandServo = null;
    private Telemetry telemetry = null;
    private double currentServoPosition = 0.0;

    // --- Tunable Constants via FTC Dashboard ---
    private static double EXTENDED_POSITION = 1.0;
    private static double RETRACTED_POSITION = 0.37;
    private static boolean TELEM = false;

    // --- State Management ---
    public enum State {EXTENDED, RETRACTED}

    private Kickstand.State currentState = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            kickstandServo = hwMap.servo.get("kickstand");
        } catch (RuntimeException e) {
            telemetry.addData("Servo \"kickstand\" not found", 0);
        }
        retract();
    }

    // --- High-Level Commands ---

    private void extend() {
        currentState = Kickstand.State.EXTENDED;
    }

    private void retract() {
        currentState = Kickstand.State.RETRACTED;
    }

    /**
     * ]
     * If kickstand is retracted, extend. If extended, retract.
     */
    public void switchState() {
        State state = currentState;
        switch (state) {
            case EXTENDED:
                retract();
                break;
            case RETRACTED:
                extend();
                break;
        }

    }

    // --- State Checking ---

    public boolean isRetracted() {
        return Math.abs(currentServoPosition - RETRACTED_POSITION) < 0.05;
    }

    public void update() {
//        currentServoPosition = kickstandServo.getPosition();

        if (currentState != null) {
            switch (currentState) {
                case EXTENDED:
//                    kickstandServo.setPosition(EXTENDED_POSITION);
                    break;
                case RETRACTED:
//                    kickstandServo.setPosition(RETRACTED_POSITION);
                    break;
            }
        }

        if (TELEM) {
            telemetry.addData("Kickstand State", currentState);
            telemetry.addData("Kickstand Servo Pos", currentServoPosition);
        }
    }
}
