package org.firstinspires.ftc.teamcode.vision.pipelineEOCV;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class TSEDetection implements VisionProcessor {

    Telemetry telemetry;

    public TSEDetection(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public Rect rect = new Rect(40, 40, 80, 200);
    public Rect rect2 = new Rect(300, 300, 400, 400);
    public Rect rect3 = new Rect(40, 50, 100, 120);
    Mat submat;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        telemetry.addData("avg h rect 2: ", avgScalar(frame, rect2).val[0]);
        telemetry.addData("avg sat rect 2: ", avgScalar(frame, rect2).val[1]);
        telemetry.addData("avg v rect 2: ", avgScalar(frame, rect2).val[2]);

        telemetry.update();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(4 * scaleCanvasDensity);

        Paint textPaint = new Paint();
        textPaint.setColor(Color.BLACK);
        textPaint.setTextSize(12);

        Paint otherPaint = new Paint();
        otherPaint.setColor(Color.GREEN);
        otherPaint.setStyle(Paint.Style.STROKE);
        otherPaint.setStrokeWidth(4 * scaleCanvasDensity);


        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(rect2, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(rect3, scaleBmpPxToCanvasPx), otherPaint);
    }

    public android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }

    private Scalar avgScalar(Mat input, Rect rect){
        submat = input.submat(rect);
        return Core.mean(submat);
    }


}
