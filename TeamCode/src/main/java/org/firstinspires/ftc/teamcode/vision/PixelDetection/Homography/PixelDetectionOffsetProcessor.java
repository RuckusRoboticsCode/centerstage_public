package org.firstinspires.ftc.teamcode.vision.PixelDetection.Homography;

import static org.opencv.core.Core.sqrt;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;

public class PixelDetectionOffsetProcessor implements VisionProcessor {

    Mat homography = new Mat(); // assumes norm of column 1 is 1
    Point pixelPoint = new Point();

    Mat pixelVec = new Mat();
    Mat displacement = new Mat();

    Mat col1 = new Mat();
    Mat col2 = new Mat();
    Mat col3 = new Mat();
    Mat tvec = new Mat();
    Mat R = new Mat(3, 3, CvType.CV_64F);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // find point on pixel

        pixelVec = new Mat(3, 1, CvType.CV_64F);
        pixelVec.put(0 ,0, pixelPoint.x);
        pixelVec.put(1, 0, pixelPoint.y);
        pixelVec.put(2, 0, 1);

//        Core.gemm

//        displacement = homography * pixelVec;

//        col1 = homography.col(0);
//        col2 = homography.col(1);
//        col3 = col1.cross(col2);
//
//        tvec = homography.col(2);
//        R = new Mat(3, 3, CvType.CV_64F);
//
//        for (int i = 0; i < 3; i++) {
//            R.put(i, 0, col1.get(i, 0)[0]);
//            R.put(i, 1, col2.get(i, 0)[0]);
//            R.put(i, 2, col3.get(i, 0)[0]);
//        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
