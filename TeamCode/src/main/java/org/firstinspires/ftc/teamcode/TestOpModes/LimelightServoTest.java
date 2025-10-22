package org.firstinspires.ftc.teamcode.TestOpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LimelightServoTest extends OpMode{
    Limelight3A limelight = null;
    Servo servo = null;
    double position = 0;
    double servodisplace = 0;
    double tx = 0; // How far left or right the target is (degrees)
    double ty = 0; // How far up or down the target is (degrees)
    double ta = 0;

    public void init(){
        servo = hardwareMap.get(Servo.class, "limelightservo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(4); // Switch to pipeline number 4


    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)
            servodisplace = tx/(360*2);


            telemetry.addData("Target X", tx);

            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.addData("servodisplace", servodisplace);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        servo.setPosition(position + servodisplace );

    }

}
