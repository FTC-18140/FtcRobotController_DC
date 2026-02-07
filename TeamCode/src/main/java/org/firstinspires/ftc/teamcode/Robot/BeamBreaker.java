package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BeamBreaker {
    private Telemetry telemetry;
    private boolean detected = false;
    DigitalChannel beamBreakSensor = null;
    public static boolean TELEM = true;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            beamBreakSensor = hwMap.digitalChannel.get("beamBreak");
        } catch (Exception e) {
            addTelemetry("Error", "Could not find digital channel 'beamBreak'");
        }
    }

    public void update() {
        if(beamBreakSensor != null) {
            detected = !beamBreakSensor.getState();
            addTelemetry("Beam Break sensor triggered: ", detected);
        }
    }

    public boolean ballDetected(){return detected;}

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
