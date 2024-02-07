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
import org.firstinspires.ftc.teamcode.helper.IMUControl;
import org.firstinspires.ftc.teamcode.auto.util.RoadRunnerLocations.RedLocations;
import org.firstinspires.ftc.teamcode.auto.util.SlidesSRLAuto;
import org.firstinspires.ftc.teamcode.auto.util.VisionController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class RedFarAuto implements AutonomousOpMode {
    private VisionController visionController;
    private CustomTypes.PropLocation propLocation;
    private SlidesSRLAuto slides;
    private DepositAuto deposit;
    private SampleMecanumDrive drive;
    private IMUControl imuControl;

    private int targetPosition = 0;
    private Pose2d startingPose;

    private int propNumber;
    private final int FIRST_PIXEL_HEIGHT = 550;
    private final int CYCLE_HEIGHT_1 = 1000;
    private final double CYCLE_OFFSET = 2.0;
    private int cycles = 0;
    private int MAX_CYCLES;
    private final int delay;
    private final boolean parkInCorner;
    private final double DEPOSIT_TIME = 0.5;

    private AutonomousStates previousState = null;
    private AutonomousStates currentState = AutonomousStates.INIT;
    private ElapsedTime timer;
    private ElapsedTime runtime;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final ArrayList<TrajectorySequence> spikeMark = new ArrayList<>();
    private final ArrayList<TrajectorySequence> backdropPreLoad = new ArrayList<>();

    private final ArrayList<TrajectorySequence> backdropPreLoadEndToStack = new ArrayList<>();
    private final ArrayList<TrajectorySequence> stackToBackdropCycle1 = new ArrayList<>();
    private final ArrayList<TrajectorySequence> forwardFromCycle1 = new ArrayList<>();

    private final ArrayList<TrajectorySequence> preLoadParkingCorner = new ArrayList<>();
    private final ArrayList<TrajectorySequence> cycle1ParkingCorner = new ArrayList<>();

    private final ArrayList<TrajectorySequence> preLoadParkingMiddle = new ArrayList<>();
    private final ArrayList<TrajectorySequence> cycle1ParkingMiddle = new ArrayList<>();

    public RedFarAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 1, false, 15);
    }

    public RedFarAuto(HardwareMap hardwareMap, Telemetry telemetry,
                      int MAX_CYCLES, boolean parkInCorner, int delay) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.MAX_CYCLES = MAX_CYCLES;
        this.parkInCorner = parkInCorner;
        this.MAX_CYCLES++;
        this.delay = delay;
    }

    @Override
    public void initialize() {
        initHardware();
        createTrajectories();
        previousState = AutonomousStates.INIT;
        currentState = AutonomousStates.START;
    }

    @Override
    public void runLoop() {

        switch (currentState) {
            case START:

                // TODO: flip left and right for blue side
                if(previousState == AutonomousStates.INIT) {
                    runtime.reset();
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
                    visionController.enableAprilTags();

                    previousState = AutonomousStates.START;
                    currentState = AutonomousStates.DRIVE_TO_SPIKE_MARK;
                }
                break;
            case DRIVE_TO_SPIKE_MARK:
                if(previousState == AutonomousStates.START) {
                    // start trajectory
                    drive.followTrajectorySequenceAsync(spikeMark.get(propNumber));
                    previousState = AutonomousStates.DRIVE_TO_SPIKE_MARK;
                } else if (previousState == AutonomousStates.DRIVE_TO_SPIKE_MARK && !drive.isBusy()) {
                    currentState = AutonomousStates.DELAY;
                }
                break;
            case DELAY:
                if(previousState != AutonomousStates.DELAY) {
                    timer.reset();
                    previousState = AutonomousStates.DELAY;
                } else if (timer.seconds() > delay){
                    currentState = AutonomousStates.DRIVE_TO_BACKDROP;
                }
                break;
            case DRIVE_TO_BACKDROP:
                if(previousState == AutonomousStates.DELAY) {
                    // start trajectory
                    drive.followTrajectorySequenceAsync(backdropPreLoad.get(propNumber));
                    previousState = AutonomousStates.DRIVE_TO_BACKDROP;
                } else if(previousState == AutonomousStates.DRIVE_TO_BACKDROP && !drive.isBusy()) {
                    currentState = AutonomousStates.SCORE_ON_BACKDROP;
                }
                break;
            case SCORE_ON_BACKDROP:
                if(previousState != AutonomousStates.SCORE_ON_BACKDROP && slides.atTarget()) {
                    timer.reset();
                    deposit.deposit();
                    previousState = AutonomousStates.SCORE_ON_BACKDROP;
                    cycles++;
                } else if(previousState == AutonomousStates.SCORE_ON_BACKDROP && timer.seconds() > DEPOSIT_TIME) {
                    currentState = AutonomousStates.DRIVE_FORWARD_FROM_BACKDROP;
                }
                break;
            case DRIVE_FORWARD_FROM_BACKDROP:
                if(previousState != AutonomousStates.DRIVE_FORWARD_FROM_BACKDROP) {
                    if(runtime.seconds() < 20 && cycles == 2) {
                        drive.followTrajectorySequenceAsync(forwardFromCycle1.get(propNumber));
                    }
                    previousState = AutonomousStates.DRIVE_FORWARD_FROM_BACKDROP;
                } else if(!drive.isBusy()) {
                    if(runtime.seconds() < 20 && cycles < MAX_CYCLES) {
                        if (cycles != 1) {
                            currentState = AutonomousStates.RELOCALIZE;
                        } else {
                            currentState = AutonomousStates.DRIVE_TO_CYCLE_START_AND_STACK;
                        }
                    } else {
                        currentState = AutonomousStates.PARK;
                    }
                }
                break;
            case RELOCALIZE:
                if (previousState == AutonomousStates.DRIVE_FORWARD_FROM_BACKDROP) {
                    // read april tag position, get pose estimate
                    Pose2d newPose = visionController.relocalize(drive.getPoseEstimate(),
                            checkIMUError(imuControl.getHeading(), drive.getPoseEstimate().getHeading()));
                    drive.setPoseEstimate(newPose);
                    previousState = AutonomousStates.RELOCALIZE;
                } else if (previousState == AutonomousStates.RELOCALIZE){
                    currentState = AutonomousStates.DRIVE_TO_CYCLE_START_AND_STACK;
                }
                break;
            case DRIVE_TO_CYCLE_START_AND_STACK:
                if(previousState != AutonomousStates.DRIVE_TO_CYCLE_START_AND_STACK) {
                    if(cycles == 1) {
                        drive.followTrajectorySequenceAsync(backdropPreLoadEndToStack.get(propNumber));
                    }
                    previousState = AutonomousStates.DRIVE_TO_CYCLE_START_AND_STACK;
                } else if(!drive.isBusy()) {
                    currentState = AutonomousStates.INTAKE;
                }
                break;
            case INTAKE:
                if(previousState == AutonomousStates.DRIVE_TO_CYCLE_START_AND_STACK) {
                    // intake
                    previousState = AutonomousStates.INTAKE;
                } else if (previousState == AutonomousStates.INTAKE) {
                    currentState = AutonomousStates.DRIVE_THROUGH_TRUSS_TO_BACKDROP;
                }
                break;
            case DRIVE_THROUGH_TRUSS_TO_BACKDROP:
                if(previousState == AutonomousStates.INTAKE) {
                    if(cycles == 1) {
                        drive.followTrajectorySequenceAsync(stackToBackdropCycle1.get(propNumber));
                    }
                    previousState = AutonomousStates.DRIVE_THROUGH_TRUSS_TO_BACKDROP;
                } else if(!drive.isBusy() && previousState == AutonomousStates.DRIVE_THROUGH_TRUSS_TO_BACKDROP) {
                    currentState = AutonomousStates.SCORE_ON_BACKDROP;
                }
                break;
            case PARK:
                if(previousState != AutonomousStates.PARK) {
                    if (cycles == 1) {
                        drive.followTrajectorySequenceAsync(
                                (parkInCorner ? preLoadParkingCorner : preLoadParkingMiddle).get(propNumber)
                        );
                    } else if (cycles == 2) {
                        drive.followTrajectorySequenceAsync(
                                (parkInCorner ? cycle1ParkingCorner : cycle1ParkingMiddle).get(propNumber)
                        );
                    }
                    previousState = AutonomousStates.PARK;
                } else if(!drive.isBusy()) {
                    currentState = AutonomousStates.STOP;
                }
                break;
            case STOP:
                break;
        }

        drive.update();
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

        imuControl = new IMUControl(hardwareMap, telemetry, startingPose.getHeading());

        deposit = new DepositAuto(hardwareMap, telemetry);

        // TODO: flip left and right detection & color for blue side
        visionController = new VisionController(hardwareMap, false, true, false, telemetry);
        slides = new SlidesSRLAuto(hardwareMap, telemetry, Constants.SlidePositions.LIMIT.position, 0.6);

        timer = new ElapsedTime();
        runtime = new ElapsedTime();
    }

    private void createTrajectories() {

        spikeMark.add(

                // left spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToConstantHeading(RedLocations.FAR_LEFT_SPIKE.getPosition())
                        .build()
        );

        spikeMark.add(

                // middle spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToConstantHeading(RedLocations.FAR_MIDDLE_SPIKE.getPosition())
                        .build()
        );

        spikeMark.add(

                // right spike mark
                drive.trajectorySequenceBuilder(RedLocations.FAR_STARTING.getPose())
                        .lineToLinearHeading(new Pose2d(-35, -43, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(-28, -37, Math.toRadians(-120)))
                        .build()
        );

        backdropPreLoad.add(

                // left backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(0).end())
                        .strafeLeft(0.001)
                        .splineToSplineHeading(new Pose2d(-33, -48, Math.toRadians(180)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-22, -11.5, Math.toRadians(180)), Math.toRadians(0))

                        .lineToConstantHeading(new Vector2d(5, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })

                        .lineToConstantHeading(new Vector2d(20, -11.5))
                        .splineToConstantHeading(RedLocations.BACKDROP_LEFT.getPosition(), Math.toRadians(0))
                        .build()
        );

        backdropPreLoad.add(

                // middle backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(1).end())
                        .forward(6)
                        .strafeRight(17)
                        .back(20)
                        .splineToSplineHeading(new Pose2d(-37, -11.5, Math.toRadians(180)), Math.toRadians(0))

                        .lineToConstantHeading(new Vector2d(5, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })

                        .lineToConstantHeading(new Vector2d(20, -11.5))
                        .splineToConstantHeading(RedLocations.BACKDROP_MIDDLE.getPosition(), Math.toRadians(0))
                        .build()
        );

        backdropPreLoad.add(

                // right backdrop
                drive.trajectorySequenceBuilder(spikeMark.get(2).end())
                        .lineToConstantHeading(new Vector2d(-31, -37))
                        .splineToSplineHeading(new Pose2d(-34, -11.5, Math.toRadians(180)), Math.toRadians(90))

                        .lineToConstantHeading(new Vector2d(10, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = FIRST_PIXEL_HEIGHT;
                        })

                        .lineToConstantHeading(new Vector2d(30, -11.5))
                        .splineToConstantHeading(RedLocations.BACKDROP_RIGHT.getPosition(), Math.toRadians(0))

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

        backdropPreLoadEndToStack.add(

                // left
                drive.trajectorySequenceBuilder(spikeMark.get(0).end())
                        .forward(3)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .splineToSplineHeading(new Pose2d(20.0, -11.5, Math.toRadians(180.0)), Math.toRadians(180.0))
                        .lineToConstantHeading(RedLocations.STACK.getPosition())
                        .build()
        );

        backdropPreLoadEndToStack.add(

                // middle
                drive.trajectorySequenceBuilder(spikeMark.get(1).end())
                        .forward(3)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .splineToSplineHeading(new Pose2d(20, -11.5, Math.toRadians(180.0)), Math.toRadians(180.0))
                        .lineToConstantHeading(RedLocations.STACK.getPosition())
                        .build()
        );

        backdropPreLoadEndToStack.add(

                // right
                drive.trajectorySequenceBuilder(spikeMark.get(2).end())
                        .forward(3)
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .splineToSplineHeading(new Pose2d(20, -11.5, Math.toRadians(180.0)), Math.toRadians(180.0))
                        .lineToSplineHeading(new Pose2d(RedLocations.STACK.getPosition(), Math.toRadians(180.0)))
                        .build()
        );

        stackToBackdropCycle1.add(

                // left
                drive.trajectorySequenceBuilder(new Pose2d(RedLocations.STACK.getPosition(), Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(20, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = CYCLE_HEIGHT_1;
                        })
                        .splineToConstantHeading(RedLocations.BACKDROP_LEFT.getPosition()
                                .minus(new Vector2d(0, CYCLE_OFFSET)), 0)
                        .build()
        );

        stackToBackdropCycle1.add(

                // middle
                drive.trajectorySequenceBuilder(new Pose2d(RedLocations.STACK.getPosition(), Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(15, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = CYCLE_HEIGHT_1;
                        })
                        .splineToConstantHeading(RedLocations.BACKDROP_MIDDLE.getPosition()
                                .minus(new Vector2d(0, CYCLE_OFFSET)), 0)
                        .build()
        );

        stackToBackdropCycle1.add(

                // right
                drive.trajectorySequenceBuilder(new Pose2d(RedLocations.STACK.getPosition(), Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(20, -11.5))
                        .addDisplacementMarker(() -> {
                            targetPosition = CYCLE_HEIGHT_1;
                        })
                        .lineToConstantHeading(new Vector2d(30, -11.5))
                        .splineToConstantHeading(RedLocations.BACKDROP_RIGHT.getPosition()
                                .plus(new Vector2d(0, CYCLE_OFFSET)), 0)
                        .build()
        );

        forwardFromCycle1.add(

                // left
                drive.trajectorySequenceBuilder(stackToBackdropCycle1.get(0).end())
                        .forward(8)
                        .build()
        );

        forwardFromCycle1.add(

                // middle
                drive.trajectorySequenceBuilder(stackToBackdropCycle1.get(1).end())
                        .forward(8)
                        .build()
        );

        forwardFromCycle1.add(

                // right
                drive.trajectorySequenceBuilder(stackToBackdropCycle1.get(2).end())
                        .forward(8)
                        .build()
        );

        cycle1ParkingCorner.add(

                // left
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(0).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .lineToConstantHeading(new Vector2d(51.75, -59))
                        .back(10)
                        .build()
        );

        cycle1ParkingCorner.add(

                // middle
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(1).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .lineToConstantHeading(new Vector2d(51.75, -59))
                        .back(10)
                        .build()
        );

        cycle1ParkingCorner.add(

                // right
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(2).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .lineToConstantHeading(new Vector2d(51.75, -59))
                        .back(10)
                        .build()
        );

        cycle1ParkingMiddle.add(

                // left
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(0).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(29)
                        .back(10)
                        .build()
        );

        cycle1ParkingMiddle.add(

                // middle
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(1).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(23)
                        .back(10)
                        .build()
        );

        cycle1ParkingMiddle.add(

                // right
                drive.trajectorySequenceBuilder(forwardFromCycle1.get(2).end())
                        .addDisplacementMarker(this::resetSlidesDeposit)
                        .strafeRight(17)
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
