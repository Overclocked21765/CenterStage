package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.nonEOCV.PropPipelineStreamable;
import org.firstinspires.ftc.teamcode.vision.pipelineEOCV.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(group = "Vision")
public class RedDetectionOp extends OpMode {
    private PropPipelineStreamable pipeline;
    private VisionPortal portal;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pipeline = new PropPipelineStreamable(this.telemetry, PropPipeline.Location.RED);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(pipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(pipeline, 0);
    }

    @Override
    public void start(){
        portal.stopStreaming();
        portal.stopLiveView();
    }

    @Override
    public void loop() {

    }
}
