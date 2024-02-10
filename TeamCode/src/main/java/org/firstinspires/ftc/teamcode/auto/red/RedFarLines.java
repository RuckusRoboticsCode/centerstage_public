package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes;
import org.firstinspires.ftc.teamcode.auto.util.CustomTypes.AutonomousStates;
import org.firstinspires.ftc.teamcode.auto.util.DepositAuto;
import org.firstinspires.ftc.teamcode.auto.util.RoadRunnerLocations.RedLocations;
import org.firstinspires.ftc.teamcode.auto.util.SlidesSRLAuto;
import org.firstinspires.ftc.teamcode.auto.util.VisionController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class RedFarLines implements AutonomousOpMode {
    private VisionController visionController;
    private CustomTypes.PropLocation propLocation;
    private SlidesSRLAuto slides;
    private DepositAuto deposit;
    private SampleMecanumDrive drive;

    private int targetPosition = 0;
    private Pose2d startingPose;

    private int propNumber;
    private final int FIRST_PIXEL_HEIGHT = 550;
    private final boolean parkInCorner;
    private final double DELAY;
    private final double DEPOSIT_TIME = 1.5;
    private double PIXEL_SCORING_OFFSET = 1.25;
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

    public RedFarLines(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 0, true, true);
    }

    public RedFarLines(HardwareMap hardwareMap, Telemetry telemetry,
                       double DELAY, boolean parkInCorner, boolean scorePixelLeft) {
        this.telemetry = telemetry;
        this.DELAY = DELAY;
        this.hardwareMap = hardwareMap;
        this.parkInCorner = parkInCorner;
        if (!scorePixelLeft) {
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
                            propNumber = 0;
                            break;
                        case MIDDLE:
                            propNumber = 1;
                            break;
                        case RIGHT:
                            propNumber = 2;
                            break;
                    }
//                    visionController.enableAprilTags();

                    previousState = AutonomousStates.START;
                    currentState = AutonomousStates.DELAY;
                }
                break;
            case DELAY:
                if (previousState != currentState) {
                    timer.reset();
                    previousState = AutonomousStates.DELAY;
                } else if (timer.seconds() > DELAY) {
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
//                    currentState = AutonomousStates.STOP;
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
//                    currentState = AutonomousStates.PARK;
                    currentState = AutonomousStates.LIFT_SLIDES_AFTER_SCORE;
                }
                break;
            case LIFT_SLIDES_AFTER_SCORE:
                if (previousState != currentState) {
                    targetPosition = FIRST_PIXEL_HEIGHT + 250;
                    previousState = AutonomousStates.LIFT_SLIDES_AFTER_SCORE;
                } else if (slides.atTarget()) {
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

        telemetry.addData("State", currentState);
        telemetry.addData("Prop", propLocation);

        if (currentState != AutonomousStates.STOP) {
            drive.update();
        }
        slides.setTargetPosition(targetPosition);
        slides.updatePower();
        telemetry.update();
    }

    private void initHardware() {

        startingPose = RedLocations.FAR_STARTING.getPose();
        drive = new SampleMecanumDrive(hardwareMap);
        // TODO: set whether to flip or not (red is false, blue is true)
        drive.setFlipPose(false);
        drive.setPoseEstimate(startingPose);
        deposit = new DepositAuto(hardwareMap, telemetry);

        // TODO: flip left and right detection & color for blue side
        visionController = new VisionController(hardwareMap, false, true, false, telemetry);
        slides = new SlidesSRLAuto(hardwareMap, telemetry, Constants.SlidePositions.LIMIT.position, 0.6);

        timer = new ElapsedTime();
    }

    private void createTrajectories() {

        spikeMark.add(

                // left spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToConstantHeading(new Vector2d(-46, -40))
                        .lineToConstantHeading(new Vector2d(-46, -48))
                        .lineToConstantHeading(new Vector2d(-33, -48))
                        .lineToConstantHeading(new Vector2d(-35, -12.5))
                        .lineToLinearHeading(new Pose2d(-35, -12.5, Math.toRadians(180)))
//                        .turn(Math.toRadians(90))
                        .build()
        );

        spikeMark.add(

                // middle spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToConstantHeading(new Vector2d(-36, -33))
                        .lineToConstantHeading(new Vector2d(-36, -41))
                        .lineToConstantHeading(new Vector2d(-57, -41))
                        .lineToConstantHeading(new Vector2d(-57, -12.5))
                        .lineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(180)))
//                        .turn(Math.toRadians(90))
                        .build()
        );

        spikeMark.add(

                // right spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToLinearHeading(new Pose2d(-33, -42, Math.toRadians(225)))
                        .lineToLinearHeading(new Pose2d(-29, -37, Math.toRadians(200)))
                        .lineToConstantHeading(new Vector2d(-36, -46))
                        .lineToLinearHeading(new Pose2d(-47, -12.5, Math.toRadians(180)))
                        .build()
        );

        backdropPreLoad.add(

                // left backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(0).end())
                        .lineToConstantHeading(new Vector2d(36, -12.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToConstantHeading(
                                RedLocations.BACKDROP_LEFT.getPosition()
                                        .plus(new Vector2d(FORWARD_OFFSET, -1 + PIXEL_SCORING_OFFSET))
                        )
                        .build()
        );

        backdropPreLoad.add(

                // middle backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(1).end())
                        .lineToConstantHeading(new Vector2d(36, -12.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToConstantHeading(
                                RedLocations.BACKDROP_MIDDLE.getPosition()
                                        .plus(new Vector2d(FORWARD_OFFSET, PIXEL_SCORING_OFFSET))
                        )
                        .build()
        );

        backdropPreLoad.add(

                // right backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(2).end())
                        .lineToConstantHeading(new Vector2d(36, -12.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })
                        .lineToConstantHeading(
                                RedLocations.BACKDROP_RIGHT.getPosition()
                                        .plus(new Vector2d(FORWARD_OFFSET, -1 + PIXEL_SCORING_OFFSET))
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
