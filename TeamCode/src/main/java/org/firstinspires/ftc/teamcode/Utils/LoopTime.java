package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopTime {

    private final ElapsedTime loopTime = new ElapsedTime();

    public LoopTime() {}

    public void reset() {
        this.loopTime.reset();
    }

    public double get() {
        return this.loopTime.milliseconds();
    }

}
