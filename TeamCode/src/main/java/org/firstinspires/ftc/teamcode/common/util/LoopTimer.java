package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTimer extends ElapsedTime {

    private double maxLoop, minLoop;
    private Telemetry telemetry;

    public LoopTimer(Telemetry telemetry){
        super();
        this.telemetry = telemetry;
    }

    public void updateLoop(){
        double currentLoop = this.time();
        this.reset();
        maxLoop = Math.max(currentLoop, maxLoop);
        minLoop = (minLoop == 0) ? currentLoop : Math.min(currentLoop, minLoop);
        telemetry.addLine("\n--------Loop times---------------------");
        telemetry.addData("Current loop: ", currentLoop);
        telemetry.addData("Max loop: ", maxLoop);
        telemetry.addData("Min loop: ", minLoop);
    }

    public void endLoop(){
        telemetry.addData("Max loop: ", maxLoop);
        telemetry.addData("Min loop: ", minLoop);
    }

    public double getMaxLoop(){
        return this.maxLoop;
    }

    public double getMinLoop(){
        return this.minLoop;
    }
}
