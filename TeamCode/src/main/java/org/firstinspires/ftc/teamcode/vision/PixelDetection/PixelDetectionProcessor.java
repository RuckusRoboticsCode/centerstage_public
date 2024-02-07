package org.firstinspires.ftc.teamcode.vision.PixelDetection;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PixelDetectionProcessor implements VisionProcessor {

    Rect MIDDLE_RECT = new Rect(
            new Point(426 ,0),
            new Point(852, 720)
    );

    Mat HSV = new Mat();
    Mat ROI = new Mat();
    Mat yellowMat = new Mat();
    Mat purpleMat = new Mat();
    Mat greenMat = new Mat();
    Mat whiteMat = new Mat();
    Mat thresh = new Mat();
    double percUncovered = 0.0;
    int numPixels = 0;
    double threshLower = 0.15;
    double threshUpper = 0.3;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);
        ROI = HSV.submat(MIDDLE_RECT);
//        ROI = HSV;

        Scalar yellowLower = new Scalar(25, 100, 20);
        Scalar yellowUpper = new Scalar(35, 255, 255);
        Core.inRange(ROI, yellowLower, yellowUpper, yellowMat);

        Scalar greenLower = new Scalar(35, 100, 20);
        Scalar greenUpper = new Scalar(75, 255, 255);
        Core.inRange(ROI, greenLower, greenUpper, greenMat);

        Scalar purpleLower = new Scalar(125, 100, 20);
        Scalar purpleUpper = new Scalar(145, 255, 255);
        Core.inRange(ROI, purpleLower, purpleUpper, purpleMat);

        // might have to be a 2 layer threshold with a more restrictive one
        // like find white-ish colors, then boost contrast, then find white
        Scalar whiteLower = new Scalar(0, 0, 100);
        Scalar whiteUpper = new Scalar(180, 45, 255);
        Core.inRange(ROI, whiteLower, whiteUpper, whiteMat);

        Mat tempMat1 = new Mat();
        Mat tempMat2 = new Mat();
        Core.bitwise_or(yellowMat, greenMat, tempMat1);
        yellowMat.release();
        greenMat.release();

        Core.bitwise_or(purpleMat, whiteMat, tempMat2);
        purpleMat.release();
        whiteMat.release();

        Core.bitwise_or(tempMat1, tempMat2, thresh);
        tempMat1.release();
        tempMat2.release();

        this.percUncovered = 1.0 - (Core.sumElems(thresh).val[0] / MIDDLE_RECT.area() / 255);

        if(percUncovered > threshUpper) {
            numPixels = 0;
        } else if(percUncovered > threshLower) {
            numPixels = 1;
        } else {
            numPixels = 2;
        }

//        for(int i = MIDDLE_RECT.x; i < MIDDLE_RECT.x + MIDDLE_RECT.width; i++) {
//            for(int j = MIDDLE_RECT.y; j < MIDDLE_RECT.y + MIDDLE_RECT.height; j++) {
//
//            }
//        }


        Imgproc.rectangle(frame, MIDDLE_RECT, new Scalar(0, 255, 0), 5);

//        thresh.copyTo(frame);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void setMIDDLE_RECT(int topLeftX, int topLeftY, int botRightX, int botRightY) {
        MIDDLE_RECT = new Rect(
                new Point(topLeftX, topLeftY),
                new Point(botRightX, botRightY)
        );
    }

    public void setThreshLower(double threshLower) {
        this.threshLower = threshLower;
    }

    public void setThreshUpper(double threshUpper) {
        this.threshUpper = threshUpper;
    }
}
