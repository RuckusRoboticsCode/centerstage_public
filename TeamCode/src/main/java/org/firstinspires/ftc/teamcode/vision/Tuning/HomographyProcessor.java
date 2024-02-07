package org.firstinspires.ftc.teamcode.vision.Tuning;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;

public class HomographyProcessor implements VisionProcessor {

    private int width;
    private int height;
    private int count = 0;
    private Mat homography = null;

    MatOfPoint2f displacements = new MatOfPoint2f(
            new Point(),
            new Point(),
            new Point(),
            new Point()
    );

    MatOfPoint2f cameraCoords = new MatOfPoint2f(
            new Point(0, 0),
            new Point(width, 0),
            new Point(width, height),
            new Point(0, height)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (count > 0) {
            return null;
        }

        homography = Calib3d.findHomography(cameraCoords, displacements);

        count++;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Mat getHomography() {
        return this.homography;
    }
}
