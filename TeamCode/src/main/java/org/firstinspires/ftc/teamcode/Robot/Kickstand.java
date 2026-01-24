package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Kickstand {
    private Servo kickstandServo;
    private Telemetry telemetry;
    public double currentServoPosition;

    // --- Tunable Constants via FTC Dashboard ---
    public static double EXTENDED_POSITION = 1;
    public static double RETRACTED_POSITION = 0.5;
    public static boolean TELEM = false;

    // --- State Management ---
    public enum State { EXTENDED, RETRACTED }
    private Kickstand.State currentState;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        try {
            kickstandServo = hwMap.servo.get("kickstand");
        } catch (Exception e) {
            telemetry.addData("Servo \"kickstand\" not found", 0);
        }
        retract();
    }

    // --- High-Level Commands ---

    public void extend() {
        currentState = Kickstand.State.EXTENDED;
    }

    public void retract() {
        currentState = Kickstand.State.RETRACTED;
    }
    public void switchState() {
        State state = currentState;
        switch(state) {
            case EXTENDED:
                currentState = State.RETRACTED;
                break;
            case RETRACTED:
                currentState = State.EXTENDED;
                break;
        }

    }

    // --- State Checking ---

    public boolean isRetracted() {
        return Math.abs(currentServoPosition - RETRACTED_POSITION) < 0.05;
    }

    public void update() {
        currentServoPosition = kickstandServo.getPosition();

        // Refactored to have a single exit point
        if (currentState != null) {
            switch (currentState) {
                case EXTENDED:
                    kickstandServo.setPosition(EXTENDED_POSITION);
                    break;
                case RETRACTED:
                    kickstandServo.setPosition(RETRACTED_POSITION);
                    break;
            }
        }

        if ( TELEM ) {
            telemetry.addData("Kickstand State", currentState);
            telemetry.addData("Kickstand Servo Pos", currentServoPosition);
        }
    }
}
