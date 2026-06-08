package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopTime
{
    ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);

    public static double LOOP_TIME = 0.04;

    public void init()
    {
        loopTimer.reset();
    }

    public void update()
    {
        LOOP_TIME = loopTimer.seconds();
        loopTimer.reset();
    }

}
