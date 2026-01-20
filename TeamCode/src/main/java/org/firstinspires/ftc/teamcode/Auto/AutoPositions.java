package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class AutoPositions {
    @Config
    public enum Positions {
        START_BLUE_FAR(new Vector2d(-63,16)),
        START_BLUE_DEPOT(new Vector2d(58,46)),
        START_RED_FAR(new Vector2d(-63,-16)),
        START_RED_DEPOT(new Vector2d(58,-46)),
        FAR_LAUNCH_ZONE_BLUE(new Vector2d(-52, 12)),
        FAR_LAUNCH_ZONE_RED(new Vector2d(-52, -12)),
        CLOSE_LAUNCH_ZONE_BLUE(new Vector2d(24, 24)),
        CLOSE_LAUNCH_ZONE_RED(new Vector2d(24, -24)),
        ARTIFACT_BASE_BLUE(new Vector2d(-34.5, 32)),
        ARTIFACT_BASE_RED(new Vector2d(-34.5, -32)),
        ARTIFACT_CENTER_BLUE(new Vector2d(-10, 32)),
        ARTIFACT_CENTER_RED(new Vector2d(-10, -32)),
        ARTIFACT_GATE_BLUE(new Vector2d(13.5, 32)),
        ARTIFACT_GATE_RED(new Vector2d(13.5, -32)),
        GATE_RED(new Vector2d(2, -52.5)),
        GATE_BLUE(new Vector2d(2, 52.5)),
        ASCENT_ZONE(new Vector2d(-28,-9.5)),
        OBSERVATION_ZONE(new Vector2d(58,-59));
        public final Vector2d position;
        Positions(Vector2d pos){
            position = pos;
        }
    }
}
