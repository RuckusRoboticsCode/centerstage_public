package org.firstinspires.ftc.teamcode.auto.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes.AutonomousStates;
import org.firstinspires.ftc.teamcode.auto.util.DepositAuto;
import org.firstinspires.ftc.teamcode.auto.util.RoadRunnerLocations;
import org.firstinspires.ftc.teamcode.auto.util.RoadRunnerLocations.RedLocations;
import org.firstinspires.ftc.teamcode.auto.util.SlidesSRLAuto;
import org.firstinspires.ftc.teamcode.auto.util.VisionController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class BlueCloseLines implements AutonomousOpMode {
    private VisionController visionController;
    private CustomTypes.PropLocation propLocation;
    private SlidesSRLAuto slides;
    private DepositAuto deposit;
    private SampleMecanumDrive drive;

    private int targetPosition = 0;
    private Pose2d startingPose;

    private int propNumber;
    private final int FIRST_PIXEL_HEIGHT = 750;
    private final boolean parkInCorner;
    private final double DEPOSIT_TIME = 1.5;
    private double PIXEL_SCORING_OFFSET = 0.0;
    private final double FORWARD_FROM_PIXEL = 8;
    private final double FORWARD_OFFSET = -1.5;

    private AutonomousStates previousState = null;
    private AutonomousStates currentState = AutonomousStates.INIT;
    private ElapsedTime timer;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final ArrayList<TrajectorySequence> spikeMark = new ArrayList<>();
    private final ArrayList<TrajectorySequence> backdropPreLoad = new ArrayList<>();;
    private final ArrayList<TrajectorySequence> preLoadParkingCorner = new ArrayList<>();
    private final ArrayList<TrajectorySequence> preLoadParkingMiddle = new ArrayList<>();

    public BlueCloseLines(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true, true);
    }

    public BlueCloseLines(HardwareMap hardwareMap, Telemetry telemetry, boolean parkInCorner, boolean scorePixelLeft) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.parkInCorner = parkInCorner;
        if (scorePixelLeft) {
            this.PIXEL_SCORING_OFFSET *= -1;
        }
    }

    @Override
    public void initialize() {
        initHardware();
        createTrajectories();
        telemetry.addData("Trajectories", "built");
        telemetry.update();
        previousState = AutonomousStates.INIT;
        currentState = AutonomousStates.START;
    }

    @Override
    public void runLoop() {

        switch (currentState) {
            case START:

                // TODO: flip left and right for blue side
                if(previousState == AutonomousStates.INIT) {
                    propLocation = visionController.getPropLocation();

                    switch (propLocation) {
                        case LEFT:
                            propNumber = 2;
                            break;
                        case MIDDLE:
                            propNumber = 1;
                            break;
                        case RIGHT:
                            propNumber = 0;
                            break;
                    }
//                    visionController.enableAprilTags();

                    previousState = AutonomousStates.START;
                    currentState = AutonomousStates.DRIVE_TO_SPIKE_MARK;
                }
                break;
            case DRIVE_TO_SPIKE_MARK:
                if(previousState == AutonomousStates.START) {
                    // start trajectory
                    drive.followTrajectorySequenceAsync(spikeMark.get(propNumber));
                    previousState = AutonomousStates.DRIVE_TO_SPIKE_MARK;
                } else if (!drive.isBusy()) {
                    currentState = AutonomousStates.DRIVE_TO_BACKDROP;
                }
                break;
            case DRIVE_TO_BACKDROP:
                if(previousState == AutonomousStates.DRIVE_TO_SPIKE_MARK) {
                    // start trajectory
                    drive.followTrajectorySequenceAsync(backdropPreLoad.get(propNumber));
                    deposit.deposit();
                    previousState = AutonomousStates.DRIVE_TO_BACKDROP;
                } else if(!drive.isBusy()) {
                    currentState = AutonomousStates.SCORE_ON_BACKDROP;
                }
                break;
            case SCORE_ON_BACKDROP:
                if(previousState != currentState && slides.atTarget()) {
                    timer.reset();
                    previousState = AutonomousStates.SCORE_ON_BACKDROP;
                } else if(previousState == AutonomousStates.SCORE_ON_BACKDROP && timer.seconds() > DEPOSIT_TIME) {
                    currentState = AutonomousStates.DROP_PIXEL;
                }
                break;
            case DROP_PIXEL:
                if(previousState != currentState) {
                    deposit.release();
                    timer.reset();
                    previousState = AutonomousStates.DROP_PIXEL;
                } else if (previousState == AutonomousStates.DROP_PIXEL && timer.seconds() > DEPOSIT_TIME) {
                    currentState = AutonomousStates.PARK;
                }
                break;
            case PARK:
                if(previousState != currentState) {
                    if (parkInCorner) {
                        drive.followTrajectorySequenceAsync(preLoadParkingCorner.get(propNumber));
                    } else {
                        drive.followTrajectorySequenceAsync(preLoadParkingMiddle.get(propNumber));
                    }
                    previousState = AutonomousStates.PARK;
                } else if(!drive.isBusy()) {
                    currentState = AutonomousStates.STOP;
                }
                break;
            case STOP:
                break;
        }

        telemetry.addData("Prop Location", propLocation);

        if (currentState != AutonomousStates.STOP) {
            drive.update();
        }
        slides.setTargetPosition(targetPosition);
        slides.updatePower();
        telemetry.update();
    }

    private void initHardware() {

        startingPose = RoadRunnerLocations.BlueLocations.CLOSE_STARTING.getPose();
        drive = new SampleMecanumDrive(hardwareMap);
        // TODO: set whether to flip or not (red is false, blue is true)
        drive.setFlipPose(true);
        drive.setPoseEstimate(startingPose);
        deposit = new DepositAuto(hardwareMap, telemetry);

        // TODO: flip left and right detection & color for blue side
        visionController = new VisionController(hardwareMap, false, false, false, telemetry);
        slides = new SlidesSRLAuto(hardwareMap, telemetry, Constants.SlidePositions.LIMIT.position, 0.6);

        timer = new ElapsedTime();
    }

    private void createTrajectories() {

        spikeMark.add(

                // left spike mark
                drive.trajectorySequenceBuilder(RoadRunnerLocations.BlueLocations.CLOSE_STARTING.getPose())
                        .lineToLinearHeading(new Pose2d(15, -43, Math.toRadians(-45)))
                        .lineToLinearHeading(
                                new Pose2d(10.5, -34.5, Math.toRadians(-30))
                                        .plus(new Pose2d(-1, 0, 0))
                        )

                        .forward(FORWARD_FROM_PIXEL)
                        .build()
        );

        spikeMark.add(

                // middle spike mark
                drive.trajectorySequenceBuilder(RoadRunnerLocations.BlueLocations.CLOSE_STARTING.getPose())
                        .lineToConstantHeading(RedLocations.CLOSE_MIDDLE_SPIKE.getPosition())

                        .forward(FORWARD_FROM_PIXEL)
                        .build()
        );

        spikeMark.add(

                // right spike mark
                drive.trajectorySequenceBuilder(RoadRunnerLocations.BlueLocations.CLOSE_STARTING.getPose())
                        .lineToConstantHeading(RedLocations.CLOSE_RIGHT_SPIKE.getPosition())

                        .forward(FORWARD_FROM_PIXEL)
                        .build()
        );

        backdropPreLoad.add(

                // left backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(0).end())
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToLinearHeading(
                                new Pose2d(RedLocations.BACKDROP_LEFT.getPosition(), Math.toRadians(180))
                                        .plus(new Pose2d(-10, 0, 0))
                        )
                        .lineToLinearHeading(
                                new Pose2d(RedLocations.BACKDROP_LEFT.getPosition(), Math.toRadians(180))
                                        .plus(new Pose2d(FORWARD_OFFSET, -1 + PIXEL_SCORING_OFFSET, 0))
                        )
                        .build()
        );
        backdropPreLoad.add(

                // middle backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(1).end())
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToLinearHeading(new Pose2d(14, -40, Math.toRadians(225)))
                        .lineToLinearHeading(
                                new Pose2d(RedLocations.BACKDROP_MIDDLE.getPosition(), Math.toRadians(180))
                                        .plus(new Pose2d(FORWARD_OFFSET, PIXEL_SCORING_OFFSET, 0))
                        )
                        .build()
        );

        backdropPreLoad.add(

                // right backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(2).end())
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToSplineHeading(new Pose2d(28, -46, Math.toRadians(225)))
                        .lineToLinearHeading(
                                new Pose2d(RedLocations.BACKDROP_RIGHT.getPosition(), Math.toRadians(180))
                                        .plus(new Pose2d(FORWARD_OFFSET, -1 + PIXEL_SCORING_OFFSET, 0))
                        )
                        .build()
        );

        preLoadParkingCorner.add(

                // left
                drive.trajectorySequenceBuilder(backdropPreLoad.get(0).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeLeft(29)
                        .back(10)
                        .build()
        );

        preLoadParkingCorner.add(

                // middle
                drive.trajectorySequenceBuilder(backdropPreLoad.get(1).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeLeft(23)
                        .back(10)
                        .build()
        );

        preLoadParkingCorner.add(

                // right
                drive.trajectorySequenceBuilder(backdropPreLoad.get(2).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeLeft(17)
                        .back(10)
                        .build()
        );

        preLoadParkingMiddle.add(

                // left
                drive.trajectorySequenceBuilder(backdropPreLoad.get(0).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(17)
                        .back(10)
                        .build()
        );

        preLoadParkingMiddle.add(

                // middle
                drive.trajectorySequenceBuilder(backdropPreLoad.get(1).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(23)
                        .back(10)
                        .build()
        );

        preLoadParkingMiddle.add(

                // right
                drive.trajectorySequenceBuilder(backdropPreLoad.get(2).end())
                        .forward(5)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(29)
                        .back(10)
                        .build()
        );
    }

    public void resetSlidesDeposit() {
        targetPosition = 0;
        deposit.intake();
    }

    private double checkIMUError(double imuHeading, double robotHeading) {
        if (Math.abs(imuHeading - robotHeading) > Math.toRadians(CustomTypes.ALLOWABLE_HEADING_ERROR)) {
            return imuHeading;
        }
        return robotHeading;
    }
}
