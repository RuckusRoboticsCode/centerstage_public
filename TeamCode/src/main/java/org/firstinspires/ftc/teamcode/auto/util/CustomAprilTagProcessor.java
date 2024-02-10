package org.firstinspires.ftc.teamcode.auto.util;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.helper.CustomAprilTagLibrary;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

public class CustomAprilTagProcessor extends AprilTagProcessorImpl implements CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private static AprilTagLibrary cvtATagLib(CustomAprilTagLibrary customAprilTagLibrary) {

        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();

        for (AprilTagMetadata metadata : customAprilTagLibrary.data) {
            builder.addTag(metadata);
        }

        return builder.build();
    }

    public CustomAprilTagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, CustomAprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, cvtATagLib(tagLibrary), drawAxes, drawCube, drawOutline, drawTagID, tagFamily, threads);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        super.init(width, height, calibration);
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return super.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public static class Builder
    {
        private double fx, fy, cx, cy;
        private TagFamily tagFamily = TagFamily.TAG_36h11;
        private CustomAprilTagLibrary tagLibrary = CustomAprilTagLibrary.getCenterStageTagLibrary();
        private DistanceUnit outputUnitsLength = DistanceUnit.INCH;
        private AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
        private int threads = THREADS_DEFAULT;

        private boolean drawAxes = false;
        private boolean drawCube = false;
        private boolean drawOutline = true;
        private boolean drawTagId = true;

        /**
         * Set the camera calibration parameters (needed for accurate 6DOF pose unless the
         * SDK has a built in calibration for your camera)
         * @param fx see opencv 8 parameter camera model
         * @param fy see opencv 8 parameter camera model
         * @param cx see opencv 8 parameter camera model
         * @param cy see opencv 8 parameter camera model
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setLensIntrinsics(double fx, double fy, double cx, double cy)
        {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }

        /**
         * Set the tag family this detector will be used to detect (it can only be used
         * for one tag family at a time)
         * @param tagFamily the tag family this detector will be used to detect
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setTagFamily(TagFamily tagFamily)
        {
            this.tagFamily = tagFamily;
            return this;
        }

        /**
         * Inform the detector about known tags. The tag library is used to allow solving
         * for 6DOF pose, based on the physical size of the tag. Tags which are not in the
         * library will not have their pose solved for
         * @param tagLibrary a library of known tags for the detector to use when trying to solve pose
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setTagLibrary(CustomAprilTagLibrary tagLibrary)
        {
            this.tagLibrary = tagLibrary;
            return this;
        }

        /**
         * Set the units you want translation and rotation data provided in, inside any
         * {@link AprilTagPoseRaw} or {@link AprilTagPoseFtc} objects
         * @param distanceUnit translational units
         * @param angleUnit rotational units
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit)
        {
            this.outputUnitsLength = distanceUnit;
            this.outputUnitsAngle = angleUnit;
            return this;
        }

        /**
         * Set whether to draw a 3D crosshair on the tag (what Vuforia did)
         * @param drawAxes whether to draw it
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setDrawAxes(boolean drawAxes)
        {
            this.drawAxes = drawAxes;
            return this;
        }

        /**
         * Set whether to draw a 3D cube projecting from the tag
         * @param drawCube whether to draw it lol
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setDrawCubeProjection(boolean drawCube)
        {
            this.drawCube = drawCube;
            return this;
        }

        /**
         * Set whether to draw a 2D outline around the tag detection
         * @param drawOutline whether to draw it
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setDrawTagOutline(boolean drawOutline)
        {
            this.drawOutline = drawOutline;
            return this;
        }

        /**
         * Set whether to annotate the tag detection with its ID
         * @param drawTagId whether to annotate the tag with its ID
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setDrawTagID(boolean drawTagId)
        {
            this.drawTagId = drawTagId;
            return this;
        }

        /**
         * Set the number of threads the tag detector should use
         * @param threads the number of threads the tag detector should use
         * @return the {@link AprilTagProcessor.Builder} object, to allow for method chaining
         */
        public CustomAprilTagProcessor.Builder setNumThreads(int threads)
        {
            this.threads = threads;
            return this;
        }

        /**
         * Create a {@link VisionProcessor} object which may be attached to
         * a {@link org.firstinspires.ftc.vision.VisionPortal} using
         * {@link org.firstinspires.ftc.vision.VisionPortal.Builder#addProcessor(VisionProcessor)}
         * @return a {@link VisionProcessor} object
         */
        public CustomAprilTagProcessor build()
        {
            if (tagLibrary == null)
            {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag library!");
            }

            if (tagFamily == null)
            {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag family!");
            }

            return new CustomAprilTagProcessor(
                    fx, fy, cx, cy,
                    outputUnitsLength, outputUnitsAngle, tagLibrary,
                    drawAxes, drawCube, drawOutline, drawTagId,
                    tagFamily, threads
            );
        }
    }
}