package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.red.RedCloseLines;
import org.firstinspires.ftc.teamcode.auto.red.RedFarLines;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.helper.LinearOpModeEx;

import java.util.List;

@Autonomous(name="Red Auto", group="Auto")
public class RedAuto extends LinearOpModeEx {

    AutonomousOpMode autonomousOpMode = null;
    ElapsedTime runtime;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private Boolean closeStartingPosition = null;
    private Boolean parkInCorner = null;
    private int waitSecondsAfterSpikeMark = 15;
    private Boolean finishedSelectingDelay = false;
    private Boolean scoreLeft = null;

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        runtime.reset();
        while (opModeInInit() && !isStopRequested() &&
                runtime.seconds() < 60 && autonomousOpMode == null) {
            clearBulkCache();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (closeStartingPosition == null){
                telemetry.clearAll();
                telemetry.addLine("Are you starting close to the backdrop?");
                telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y) || gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                    closeStartingPosition = true;
                } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B) || gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                    closeStartingPosition = false;
                }
            } else if (parkInCorner == null){
                telemetry.clearAll();
                telemetry.addLine("Should autonomous mode park in the corner or in the middle?");
                telemetry.addLine("Press (Y/Δ) for corner, (B/O) for middle");

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y) || gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                    parkInCorner = true;
                } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B) || gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                    parkInCorner = false;
                }
            } else if (!finishedSelectingDelay && !closeStartingPosition){
                telemetry.clearAll();
                telemetry.addLine("How long should the autonomous mode wait before scoring on backdrop?");
                telemetry.addLine("Press DPAD UP and DOWN to adjust the time.");
                telemetry.addLine("Press (Y/Δ) to confirm");
                telemetry.addData("Delay (s): ", waitSecondsAfterSpikeMark);

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    waitSecondsAfterSpikeMark++;
                } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    waitSecondsAfterSpikeMark--;
                }

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y) || gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                    finishedSelectingDelay = true;
                }

                if (waitSecondsAfterSpikeMark > 15) {
                    waitSecondsAfterSpikeMark = 15;
                } else if (waitSecondsAfterSpikeMark < 0) {
                    waitSecondsAfterSpikeMark = 0;
                }
            } else if (scoreLeft == null) {
                telemetry.clearAll();
                telemetry.addLine("Would you like to score on the left or right?");
                telemetry.addLine("Press the corresponding DPAD");

                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    scoreLeft = true;
                } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    scoreLeft = false;
                }

            } else {
                if (closeStartingPosition) {
                    autonomousOpMode = new RedCloseLines(hardwareMap, telemetry, parkInCorner, scoreLeft);
                    break;
                }
                autonomousOpMode = new RedFarLines(hardwareMap, telemetry, waitSecondsAfterSpikeMark, parkInCorner, scoreLeft);
                break;
            }

            telemetry.update();
        }

        if (autonomousOpMode == null) {
            autonomousOpMode = new RedCloseLines(hardwareMap, telemetry);
        }

        autonomousOpMode.initialize();

        waitForStart();

        while (opModeIsActive()) {
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
