package org.firstinspires.ftc.teamcode.vision.PixelDetection;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DistanceDetectionProcessor implements VisionProcessor {

    private final Rect MIDDLE_RECT = new Rect(
            new Point(426 ,0),
            new Point(852, 720)
    );

    private Mat HSV = new Mat();
    private Mat ROI = new Mat();
    private Mat yellowMat = new Mat();
    private Mat purpleMat = new Mat();
    private Mat greenMat = new Mat();
    private Mat whiteMat = new Mat();
    private Mat thresh = new Mat();
    private double percUncovered = 0.0;
    private int numPixels = 0;
    private final double threshLower = 0.15;
    private final double threshUpper = 0.3;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);
        ROI = HSV.submat(MIDDLE_RECT);

        Scalar yellowLower = new Scalar(25, 100, 20);
        Scalar yellowUpper = new Scalar(35, 255, 255);
        Core.inRange(ROI, yellowLower, yellowUpper, yellowMat);
        processImage(yellowMat);

        Scalar greenLower = new Scalar(35, 100, 20);
        Scalar greenUpper = new Scalar(75, 255, 255);
        Core.inRange(ROI, greenLower, greenUpper, greenMat);
        processImage(greenMat);

        Scalar purpleLower = new Scalar(125, 100, 20);
        Scalar purpleUpper = new Scalar(145, 255, 255);
        Core.inRange(ROI, purpleLower, purpleUpper, purpleMat);
        processImage(purpleMat);

        // might have to be a 2 layer threshold with a more restrictive one
        // like find white-ish colors, then boost contrast, then find white
        Scalar whiteLower = new Scalar(0, 0, 100);
        Scalar whiteUpper = new Scalar(180, 45, 255);
        Core.inRange(ROI, whiteLower, whiteUpper, whiteMat);
        processImage(whiteMat);

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

        Imgproc.rectangle(frame, MIDDLE_RECT, new Scalar(0, 255, 0), 5);

        return null;
    }

    private void processImage(Mat colorThresholdFrame) {
        Imgproc.erode(colorThresholdFrame, colorThresholdFrame,
                Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5, 5)));
        Imgproc.blur(colorThresholdFrame, colorThresholdFrame, new Size(5, 5));
    }

    private double getDistanceFromContour(Mat threshFrame) {
        Mat edges = new Mat();
        Imgproc.Canny(threshFrame, edges, 100, 300);

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

        Rect largestContour = boundRect[boundRect.length - 1];
        double percROIWidth = (double) largestContour.width / MIDDLE_RECT.width;

        double ROI_WIDTH_INCHES = 2.5;
        double EXPECTED_PIXEL_WIDTH_INCHES = 2.25;
        double EXPECTED_ROI_WIDTH_PERC = EXPECTED_PIXEL_WIDTH_INCHES / ROI_WIDTH_INCHES;

        return 0;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
