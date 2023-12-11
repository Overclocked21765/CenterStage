package org.firstinspires.ftc.teamcode.vision.pipelineEOCV;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

//import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

//@Config
public class PropPipelineEOCV implements VisionProcessor {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Side location = Side.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Rect leftZoneArea;
    private Rect centerZoneArea;

    public Rect rect = new Rect(100, 100, 125, 125);
    public Rect rect2 = new Rect(300, 300, 125, 125);

    private Mat finalMat = new Mat();

    public static int blueLeftX = 100;
    public static int blueLeftY = 300;

    public static int blueCenterX = 300;
    public static int blueCenterY = 300;

    public static int redLeftX = 100;
    public static int redLeftY = 300;

    public static int redCenterX = 300;
    public static int redCenterY = 300;

    public static int width = 125;
    public static int height = 125;

    public static double redThreshold = 2.5;
    public static double blueThreshold = 0.2;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    private Telemetry telemetry;

    public PropPipelineEOCV(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        leftZoneArea = new Rect(Globals.COLOR == Side.RED? redLeftX : blueLeftX, Globals.COLOR == Side.RED? redLeftY : blueLeftY, width, height);
        centerZoneArea = new Rect(Globals.COLOR == Side.RED?redCenterX:blueCenterX, Globals.COLOR == Side.RED?redCenterY:blueCenterY, width, height);

//        leftZoneArea = rect;
//        centerZoneArea = rect2;

        Mat leftZone = finalMat.submat(leftZoneArea);
        Mat centerZone = finalMat.submat(centerZoneArea);

        left = Core.sumElems(leftZone);
        center = Core.sumElems(centerZone);

        leftColor = left.val[0] / 1000000.0;
        centerColor = center.val[0] / 1000000.0;

        if(Globals.COLOR == Side.BLUE){
            if (leftColor < threshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor < threshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = Side.RIGHT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }else{
            if (leftColor > threshold) {
                // left zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor > threshold) {
                // center zone has it
                location = Side.RIGHT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }

        Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

//        Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
//        Utils.matToBitmap(finalMat, b);
//        lastFrame.set(b);

        leftZone.release();
        centerZone.release();

        telemetry.addLine("balls: ");
        telemetry.addData("loc: ", this.getLocation());
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


        canvas.drawRect(makeGraphicsRect(leftZoneArea, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(centerZoneArea, scaleBmpPxToCanvasPx), rectPaint);
//        canvas.drawRect(makeGraphicsRect(rect3, scaleBmpPxToCanvasPx), otherPaint);
    }

    public android.graphics.Rect makeGraphicsRect(org.opencv.core.Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }

    public Side getLocation() {
        return this.location;
    }

//    @Override
//    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//    }

    public enum Side{
        LEFT,
        CENTER,
        RIGHT,
        RED,
        BLUE
    }

    public static class Globals{
        public static Side COLOR = Side.RED;
    }
}
