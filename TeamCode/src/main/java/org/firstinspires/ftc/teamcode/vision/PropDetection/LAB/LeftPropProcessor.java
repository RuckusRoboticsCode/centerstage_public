package org.firstinspires.ftc.teamcode.vision.PropDetection.LAB;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class LeftPropProcessor implements VisionProcessor {

    private Mat cvtMat = new Mat();
    private Mat filteredMat = new Mat();

    private double leftThreshold = 0.15;
    private double middleThreshold = 0.15;

    public Scalar blueLowerBound = new Scalar(0, 120, 100);
    public Scalar blueUpperBound = new Scalar(255, 130, 120);
    public Scalar redLowerBound = new Scalar(0, 130, 158);
    public Scalar redUpperBound = new Scalar(255, 210, 190);

    CustomTypes.PropLocation propLocation;
    double leftPerc;
    double middlePerc;

    Rect LEFT_RECTANGLE;
    Rect MIDDLE_RECTANGLE;
    private boolean detectingRed = true;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.LEFT_RECTANGLE = new Rect(
                new Point(0, 0),
                new Point(width / 3, height)
        );

        this.MIDDLE_RECTANGLE = new Rect(
                new Point(width / 3, 0),
                new Point(3 * width / 4, height)
        );
    }

    public void setDetectingRed(boolean detectingRed) {
        this.detectingRed = detectingRed;
    }

    public void setLeftThreshold(double threshold) {
        this.leftThreshold = threshold;
    }

    public void setMiddleThreshold(double threshold) {
        this.middleThreshold = threshold;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, cvtMat, Imgproc.COLOR_RGB2Lab);
        if (detectingRed) {
            Core.inRange(cvtMat, redLowerBound, redUpperBound, filteredMat);
        } else {
            Core.inRange(cvtMat, blueLowerBound, blueUpperBound, filteredMat);
        }
        cvtMat.release();

        double leftBox = Core.sumElems(filteredMat.submat(LEFT_RECTANGLE)).val[0];
        double middleBox = Core.sumElems(filteredMat.submat(MIDDLE_RECTANGLE)).val[0];

        this.leftPerc = leftBox / LEFT_RECTANGLE.area() / 255;
        this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]

        if(leftPerc > middlePerc
                && leftPerc > leftThreshold) {
            propLocation = CustomTypes.PropLocation.LEFT;
        } else if (middlePerc > leftPerc
                && middlePerc > middleThreshold) {
            propLocation = CustomTypes.PropLocation.MIDDLE;
        } else {
            propLocation = CustomTypes.PropLocation.RIGHT;
        }

        Scalar greenBorder = new Scalar(0, 255, 0);
        Scalar redBorder = new Scalar(255, 0, 0);

        switch (propLocation) {
            case LEFT:
                Imgproc.rectangle(frame, LEFT_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
            case MIDDLE:
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
                break;
            case RIGHT:
                Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
