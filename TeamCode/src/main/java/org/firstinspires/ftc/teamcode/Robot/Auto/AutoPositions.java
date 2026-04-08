package org.firstinspires.ftc.teamcode.Robot.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class AutoPositions {
    @Config
    public enum Positions {
        START_BLUE_FAR(new Vector2d(-63.0, 16.5)),
        START_BLUE_DEPOT(new Vector2d(58.0, 46.0)),
        START_RED_FAR(new Vector2d(-63.0, -16.5)),
        START_RED_DEPOT(new Vector2d(57.5, -45.5)),
        FAR_LAUNCH_ZONE_BLUE(new Vector2d(-56.0, 16.5)),
        FAR_LAUNCH_ZONE_RED(new Vector2d(-56.0, -16.5)),
        CLOSE_LAUNCH_ZONE_BLUE(new Vector2d(32.0, 30.0)),
        CENTER_LAUNCH_ZONE_BLUE(new Vector2d(18.0, 14.0)),
        PARKING_LAUNCH_ZONE_BLUE(new Vector2d(35.0, 12.0)),
        CLOSE_LAUNCH_ZONE_RED(new Vector2d(32.0, -30.0)),
        CENTER_LAUNCH_ZONE_RED(new Vector2d(18.0, -14.0)),
        PARKING_LAUNCH_ZONE_RED(new Vector2d(35.0, -12.0)),
        ARTIFACT_BASE_BLUE(new Vector2d(-35, 26.0)),
        ARTIFACT_BASE_RED(new Vector2d(-33.5, -26.0)),
        ARTIFACT_CENTER_BLUE(new Vector2d(-10, 24.0)),
        ARTIFACT_CENTER_RED(new Vector2d(-9.5, -24.0)),
        ARTIFACT_GATE_BLUE(new Vector2d(13.5, 26.0)),
        ARTIFACT_GATE_RED(new Vector2d(14.5, -26.0)),
        GATE_RED(new Vector2d(5.0, -54.0)),
        GATE_BLUE(new Vector2d(5.0, 54.0)),
        LOADING_ZONE_BLUE(new Vector2d(-61.0, 62.0)),
        LOADING_ZONE_RED(new Vector2d(-61.0, -62.0)),
        OBSERVATION_ZONE(new Vector2d(58.0, -59.0));
        public final Vector2d position;

        Positions(Vector2d pos) {
            position = pos;
        }
    }
}
