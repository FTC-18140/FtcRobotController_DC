package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class AutoPositions {
    @Config
    public enum Positions {
        START_LEFT(new Vector2d(-15,-60)),
        START_RIGHT(new Vector2d(15,-60)),
        BASKET(new Vector2d(-55, -55)),
        SAMPLE_1_LEFT(new Vector2d(-49, -42)),
        SAMPLE_2_LEFT(new Vector2d(-59.5, -42)),
        SAMPLE_3_LEFT(new Vector2d(-58, -42)),
        ASCENT_ZONE(new Vector2d(-28,-9.5)),
        OBSERVATION_ZONE(new Vector2d(58,-59));
        public final Vector2d position;
        Positions(Vector2d pos){
            position = pos;
        }
    }
}
