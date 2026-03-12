package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelController {

    private Telemetry telemetry = null;
    private Flywheel lowerWheel = null;
    private Flywheel upperWheel = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        lowerWheel.init(hwMap, telem, "launcher");
        upperWheel.init(hwMap, telem, "launcher2");
    }
}
