package org.firstinspires.ftc.teamcode.vision.PropDetection.HSV;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class LeftPropProcessor implements VisionProcessor, CameraStreamSource {

	private final AtomicReference<Bitmap> lastFrame =
			new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

	private Mat testMat = new Mat();
	private Mat highMat = new Mat();
	private Mat lowMat = new Mat();
	private Mat finalMat = new Mat();
	private double middleThreshold = 0.1;
	private double leftThreshold = 0.25;
	Telemetry telemetry;

	CustomTypes.PropLocation propLocation;
	double leftPerc;
	double middlePerc;

	Rect LEFT_RECTANGLE;
	Rect MIDDLE_RECTANGLE;
	private boolean detectingRed = true;

	@Override
	public void init(int width, int height, CameraCalibration calibration) {

		this.LEFT_RECTANGLE = new Rect(
				new Point(0, height / 3.5),
				new Point(2 * width / 9, 2 * height / 3)
		);

		this.MIDDLE_RECTANGLE = new Rect(
				new Point(5 * width / 16, height / 3.5),
				new Point(9 * width / 10, 2 * height / 3)
		);

		lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
	}

	public void setDetectionColor(boolean detectRed) {
		this.detectingRed = detectRed;
	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

		if(this.detectingRed) {
			Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
			Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

			Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
			Scalar highHSVRedUpper = new Scalar(180, 255, 255);

			Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
			Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);
			Core.bitwise_or(lowMat, highMat, finalMat);
		} else {
			Scalar blueHSVLower = new Scalar(85, 100, 20);
			Scalar blueHSVUpper = new Scalar(140, 255, 255);

			Core.inRange(testMat, blueHSVLower, blueHSVUpper, finalMat);
		}
//		testMat.release();
//
//		lowMat.release();
//		highMat.release();

		double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
		double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

		this.leftPerc = leftBox / LEFT_RECTANGLE.area() / 255;
		this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]
		this.middlePerc *= 2;

		if(leftPerc > middlePerc
			&& leftPerc > leftThreshold) {
			propLocation = CustomTypes.PropLocation.LEFT;
		} else if (middlePerc > leftPerc
					&& middlePerc > middleThreshold) {
			propLocation = CustomTypes.PropLocation.MIDDLE;
		} else {
			propLocation = CustomTypes.PropLocation.RIGHT;
		}

		/*This line should only be added in when you want to see your custom pipeline
		on the driver station stream, do not use this permanently in your code as
		you use the "frame" mat for all of your pipelines, such as April Tags*/
//		finalMat.copyTo(frame);
		Scalar redBorder = new Scalar(255, 0, 0);
		Scalar greenBorder = new Scalar(0, 255, 0);

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

		Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
		Utils.matToBitmap(frame, b);
		lastFrame.set(b);

		return null;

//		return null; //You do not return the original mat anymore, instead return null

	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
	}

	public CustomTypes.PropLocation getPropLocation() {
		return this.propLocation;
	}

	public double[] getPercents() {
		return new double[]{leftPerc, middlePerc};
	}

	public void setTelemetry(Telemetry telemetry) {
		this.telemetry = telemetry;
	}

	public void updateTelemetry() {
		telemetry.addLine("Prop Processor")
				.addData("Left Percent", leftPerc)
				.addData("Middle Percent", middlePerc)
				.addData("Prop Location", propLocation);
	}

	@Override
	public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
		continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
	}
}