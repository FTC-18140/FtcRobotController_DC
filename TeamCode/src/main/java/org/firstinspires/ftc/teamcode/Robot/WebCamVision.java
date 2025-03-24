package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class WebCamVision
{
    private WebcamName theCamera;
    public SamplePipeline sampleFinder;
    VisionPortal thePortal;
    Telemetry telemetry;

    void init(HardwareMap hardwareMap, Telemetry telem)
    {
        telemetry = telem;
        try
        {
            theCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        catch (Exception e)
        {
            telemetry.addData("Webcam 1 not found.", 0);
        }
        try
        {
            sampleFinder = new SamplePipeline();
            sampleFinder.setTelemetry(telemetry);
            thePortal = VisionPortal.easyCreateWithDefaults(theCamera, sampleFinder);
            thePortal.stopLiveView();
        }
        catch (Exception e)
        {
            telemetry.addData("exception:  ", e.getMessage());
        }
    }


    public double getSampleX()
    {
        if (sampleFinder != null)
        {
            return sampleFinder.xPos;
        }
        else
        {
            return -1;
        }
    }

    public double getSampleY()
    {
        if (sampleFinder != null)
        {
            return sampleFinder.yPos;
        }
        else
        {
            return -1;
        }
    }

    public void stopSampleVisionProcessor()
    {
        if (sampleFinder != null)
        {
            thePortal.setProcessorEnabled(sampleFinder, false);
        }
        else
        {
            telemetry.addData("Can't disable Sample Vision Processor. Not initialized.", 0);
        }
    }

    public void startSampleVisionProcessor()
    {
        if (sampleFinder != null)
        {
            thePortal.setProcessorEnabled(sampleFinder, true);
        }
        else
        {
            telemetry.addData("Can't enable Sample Vision Processor. Not initialized.", 0);
        }
    }


}
