package org.firstinspires.ftc.teamcode.vision.PropDetection.Contour;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PropContourLAB implements VisionProcessor {

    private Mat cvtMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat filteredMat = new Mat();
    private Mat outputMat = new Mat();
    private Mat masked = new Mat();
//    public int channels = 0;
    public Scalar blueLowerBound = new Scalar(0, 120, 100);
    public Scalar blueUpperBound = new Scalar(255, 130, 120);
    public Scalar redLowerBound = new Scalar(0, 130, 158);
    public Scalar redUpperBound = new Scalar(255, 210, 190);
    public int lowerThreshold = 200;
    public int upperThreshold = 255;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, cvtMat, Imgproc.COLOR_RGB2Lab);
//        Core.inRange(cvtMat, blueLowerBound, blueUpperBound, filteredMat);
        Core.inRange(cvtMat, redLowerBound, redUpperBound, filteredMat);

        Mat edges = new Mat();
        Imgproc.Canny(filteredMat, edges, lowerThreshold, upperThreshold);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        masked.release();
        Core.bitwise_and(frame, frame, masked, filteredMat);
        masked.copyTo(frame);

        if(boundRect.length != 0) {
            Imgproc.rectangle(frame, boundRect[boundRect.length - 1], new Scalar(0, 255, 0), 5);
        }

//        for(Rect r : boundRect) {
//            Imgproc.rectangle(frame, r, new Scalar(0, 255, 0), 5);
//        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
