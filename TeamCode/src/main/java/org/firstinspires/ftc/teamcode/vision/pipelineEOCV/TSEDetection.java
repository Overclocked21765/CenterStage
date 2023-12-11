package org.firstinspires.ftc.teamcode.vision.pipelineEOCV;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class TSEDetection implements VisionProcessor {

    Telemetry telemetry;

    private boolean hFound = false;

    private Paint textPaint, boxPaint;

    public int textSize = 30;

    public TSEDetection(Telemetry telemetry){
        this.telemetry = telemetry;

        textPaint = new Paint();
        textPaint.setColor(Color.BLACK);
        textPaint.setTypeface(Typeface.DEFAULT);
        textPaint.setTextSize(textSize);
        textPaint.setAntiAlias(true);

        boxPaint = new Paint();
        boxPaint.setColor(Color.RED);
        boxPaint.setStyle(Paint.Style.FILL);
    }

    public Rect rect = new Rect(40, 40, 80, 200);
    public Rect rect2 = new Rect(500, 300, 125, 125);
    public Rect rect3 = new Rect(40, 50, 100, 120);
    Mat submat;
    Mat hsvMat = new Mat();
    Mat displayMat = new Mat();
    Mat mask = new Mat();
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        boolean h = false, s = false, v = false;


//        Imgproc.cvtColor(displayMat, hsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_BGR2HSV);
        //Imgproc.blur(displayMat, displayMat, new Size(0.5, 0.5));

        double[] vals = avgScalar(frame, rect2).val;

        telemetry.addData("avg h rect 2: ", vals[0]);
        telemetry.addData("avg sat rect 2: ", vals[1]);
        telemetry.addData("avg v rect 2: ", vals[2]);

        h = ((vals[0] >= 170d && vals[0] <= 210d) || (vals[0] >= 0d && vals[0] <= 10d));
        this.hFound = h;
        s = vals[1] >= 70d && vals[1] <= 355d;
        v = vals[2] >= 50d && vals[2] >= 255d;

        telemetry.addData("red detected? :", (h && s && v));
        telemetry.addData("h?: ", h);

        telemetry.update();

        submat.release();
        hsvMat.release();
        displayMat.release();
        mask.release();

        return displayMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor((hFound) ? Color.GREEN : Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(4 * scaleCanvasDensity);



        Paint otherPaint = new Paint();
        otherPaint.setColor(Color.BLACK);
        otherPaint.setStyle(Paint.Style.STROKE);
        otherPaint.setStrokeWidth(4 * scaleCanvasDensity);

        this.textPaint.setTextSize(this.textSize);


        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), otherPaint);
        canvas.drawRect(makeGraphicsRect(rect2, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(rect3, scaleBmpPxToCanvasPx), otherPaint);
        canvas.drawText("" + this.hFound, 420, 250, textPaint);
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
