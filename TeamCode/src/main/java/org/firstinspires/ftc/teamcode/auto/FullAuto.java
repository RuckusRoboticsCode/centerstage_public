package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.blue.BlueCloseAuto;
import org.firstinspires.ftc.teamcode.auto.blue.BlueFarAuto;
import org.firstinspires.ftc.teamcode.auto.red.RedCloseAuto;
import org.firstinspires.ftc.teamcode.auto.red.RedFarAuto;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.tele.subsystems.DroneLauncher;

import java.util.List;

//@Photon
@Disabled
@Autonomous(name="Auto", group="Auto")
public class FullAuto extends LinearOpMode {

    private GamepadEx gamepadEx;

    private Boolean redAlliance = null;
    private Boolean closeStartingPosition = null;
    private Boolean goThroughStageDoor = null;
    private Boolean parkInCorner = null;
    private int waitSecondsAfterSpikeMark = 15;
    private Boolean finishedSelectingDelay = false;
    private int numCycles = 0;
    private Boolean finishedSelectingCycles = false;

    private AutonomousOpMode autonomousOpMode = null;
    private ElapsedTime initTimer = null;
    private DroneLauncher droneLauncher;

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() throws InterruptedException {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        droneLauncher = new DroneLauncher();
        droneLauncher.init(hardwareMap, telemetry);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.readButtons();
        initTimer = new ElapsedTime();

        while (opModeInInit() && !isStopRequested()) {

            clearBulkCache();

            if(initTimer.seconds() > 20.0) {
                // default if they take too long
                autonomousOpMode = new RedCloseAuto(hardwareMap, telemetry);
                break;
            }
            if(redAlliance == null) {
                telemetry.clearAll();
                telemetry.addLine("Are you on red alliance?");
                telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    redAlliance = true;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
                    redAlliance = false;
                }
            } else if (closeStartingPosition == null){
                telemetry.clearAll();
                telemetry.addLine("Are you starting close to the backdrop?");
                telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    closeStartingPosition = true;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
                    closeStartingPosition = false;
                }
            } else if (goThroughStageDoor == null) {
                telemetry.clearAll();
                telemetry.addLine("Should autonomous mode go through the stage door?");
                telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    goThroughStageDoor = true;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
                    goThroughStageDoor = false;
                }
            } else if (parkInCorner == null){
                telemetry.clearAll();
                telemetry.addLine("Should autonomous mode park in the corner or in the middle?");
                telemetry.addLine("Press (Y/Δ) for corner, (B/O) for middle");
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    parkInCorner = true;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
                    parkInCorner = false;
                }
            } else if (!finishedSelectingDelay){
                telemetry.clearAll();
                telemetry.addLine("How long should the autonomous mode wait before scoring on backdrop?");
                telemetry.addLine("Press (Y/Δ) to confirm");
                telemetry.addData("Delay (s): ", waitSecondsAfterSpikeMark);
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    waitSecondsAfterSpikeMark++;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    waitSecondsAfterSpikeMark--;
                }

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    finishedSelectingDelay = true;
                }

                if (waitSecondsAfterSpikeMark > 15) {
                    waitSecondsAfterSpikeMark = 15;
                } else if (waitSecondsAfterSpikeMark < 0) {
                    waitSecondsAfterSpikeMark = 0;
                }

            } else if (!finishedSelectingCycles) {
                telemetry.clearAll();
                telemetry.addLine("How many cycles should the autonomous mode attempt?");
                telemetry.addLine("Press (Y/Δ) to confirm");
                telemetry.addData("Cycles (#): ", numCycles);
                telemetry.update();

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    numCycles++;
                } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    numCycles--;
                }

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                    finishedSelectingCycles = true;
                }

                if (numCycles > 2) {
                    numCycles = 2;
                } else if (numCycles < 0) {
                    numCycles = 0;
                }
            } else {
                break;
            }
        }

        // create op mode here
        if (autonomousOpMode == null) {
            if (redAlliance) {
                if (closeStartingPosition) {
                    autonomousOpMode = new RedCloseAuto(hardwareMap, telemetry, numCycles, parkInCorner);
                } else {
                    autonomousOpMode = new RedFarAuto(hardwareMap, telemetry, numCycles, parkInCorner, waitSecondsAfterSpikeMark);
                }
            } else {
                if (closeStartingPosition) {
                    autonomousOpMode = new BlueCloseAuto(hardwareMap, telemetry, numCycles, parkInCorner);
                } else {
                    autonomousOpMode = new BlueFarAuto(hardwareMap, telemetry, numCycles, parkInCorner, waitSecondsAfterSpikeMark);
                }
            }
        }

        autonomousOpMode.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            clearBulkCache();
            autonomousOpMode.runLoop();
        }
    }

    private void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
