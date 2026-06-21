package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopTime
{
    ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.MILLIS_IN_NANO);
    MovingAverageFilter loopTimeFilter = new MovingAverageFilter(25);
    public static double LOOP_TIME = 0.04;

    public void init()
    {
        loopTimer.reset();
    }

    public void update()
    {
        LOOP_TIME = loopTimeFilter.addValue(loopTimer.seconds());
        loopTimer.reset();
    }

}
