package org.firstinspires.ftc.teamcode.nonEOCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PropDetectionOp extends OpMode {
    private PropPipeline pipeline;
    private VisionPortal portal;

    private PropPipeline.Side side;

    @Override
    public void init(){
        PropPipeline.Globals.COLOR = PropPipeline.Side.RED;

        pipeline = new PropPipeline();
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                pipeline
        );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop(){
        side = pipeline.getLocation();
        telemetry.addData("Side: ", side);

    }

    @Override
    public void start(){
        portal.stopStreaming();
        portal.close();
    }

    @Override
    public void loop(){
        telemetry.addLine("there is nothing to do anymore");
    }
}
