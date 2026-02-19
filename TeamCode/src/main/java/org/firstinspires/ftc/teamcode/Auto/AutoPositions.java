package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class AutoPositions {
    @Config
    public enum Positions {
        START_BLUE_FAR(new Vector2d(-63,16.5)),
        START_BLUE_DEPOT(new Vector2d(58,46)),
        START_RED_FAR(new Vector2d(-63,-16.5)),
        START_RED_DEPOT(new Vector2d(57.75,-45.75)),
        FAR_LAUNCH_ZONE_BLUE(new Vector2d(-56, 16.5)),
        FAR_LAUNCH_ZONE_RED(new Vector2d(-56, -16.5)),
        CLOSE_LAUNCH_ZONE_BLUE(new Vector2d(32, 30)),
        CENTER_LAUNCH_ZONE_BLUE(new Vector2d(18, 14)),
        PARKING_LAUNCH_ZONE_BLUE(new Vector2d(33, 12)),
        CLOSE_LAUNCH_ZONE_RED(new Vector2d(32, -30)),
        CENTER_LAUNCH_ZONE_RED(new Vector2d(18, -16)),
        PARKING_LAUNCH_ZONE_RED(new Vector2d(33, -12)),
        ARTIFACT_BASE_BLUE(new Vector2d(-34.5, 26)),
        ARTIFACT_BASE_RED(new Vector2d(-34.5, -26)),
        ARTIFACT_CENTER_BLUE(new Vector2d(-10, 24)),
        ARTIFACT_CENTER_RED(new Vector2d(-10, -24)),
        ARTIFACT_GATE_BLUE(new Vector2d(13.5, 26)),
        ARTIFACT_GATE_RED(new Vector2d(13.5, -26)),
        GATE_RED(new Vector2d(5, -54)),
        GATE_BLUE(new Vector2d(5, 54)),
        LOADING_ZONE_BLUE(new Vector2d(-60,62)),
        LOADING_ZONE_RED(new Vector2d(-60,-62)),
        OBSERVATION_ZONE(new Vector2d(58,-59));
        public final Vector2d position;
        Positions(Vector2d pos){
            position = pos;
        }
    }
}
