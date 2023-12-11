package org.firstinspires.ftc.teamcode.vision.pipelineEOCV;

import android.graphics.*;
import android.graphics.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TSEDetection2 extends OpenCvPipeline {
    public int blur = 0;

    private Telemetry telemetry;

    private Paint boxPaint;
    private Paint textPaint;

    public org.opencv.core.Rect rect = new org.opencv.core.Rect(40, 40, 80, 200);
    public org.opencv.core.Rect rect2 = new org.opencv.core.Rect(300, 300, 400, 400);
    public org.opencv.core.Rect rect3 = new org.opencv.core.Rect(40, 50, 100, 120);

    private Mat submat = new Mat();
    private Mat hsvMat = new Mat();

    public TSEDetection2(Telemetry telemetry) {
        this.telemetry = telemetry;

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTypeface(Typeface.DEFAULT);
        textPaint.setTextSize(30);
        textPaint.setAntiAlias(true);

        boxPaint = new Paint();
        boxPaint.setColor(Color.BLACK);
        boxPaint.setStyle(Paint.Style.FILL);
    }

    @Override
    public Mat processFrame(Mat input) {




        if (blur > 0 && blur % 2 == 1) {
            Imgproc.GaussianBlur(input, input, new Size(blur, blur), 0);
        } else if (blur > 0) {
            Imgproc.GaussianBlur(input, input, new Size(blur + 1, blur + 1), 0);
        }

//        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

//        telemetry.addData("sub h: ", this.avgScalar(hsvMat, rect2).val[0]);
//        telemetry.addData("sub s: ", this.avgScalar(hsvMat, rect2).val[1]);
//        telemetry.addData("sub v: ", this.avgScalar(hsvMat, rect2).val[2]);
        double red = rVal(input, rect2);
        telemetry.addData("rVal rect2: ", red);

        telemetry.addData("red detected: ", red < 2.5);



        telemetry.addLine("ballss");
        telemetry.update();
        return input;
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

    public android.graphics.Rect makeGraphicsRect(org.opencv.core.Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }

    private Scalar avgScalar(Mat input, org.opencv.core.Rect rect){
        submat = input.submat(rect);
        return Core.mean(submat);
    }

    private double rVal(Mat input, org.opencv.core.Rect rect){
        submat = input.submat(rect);
        return (Core.sumElems(submat).val[0]) / 1000000.0;
    }

}
