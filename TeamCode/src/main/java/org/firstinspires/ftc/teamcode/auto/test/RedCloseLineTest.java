package org.firstinspires.ftc.teamcode.auto.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.red.RedCloseLines;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.helper.LinearOpModeEx;

import java.util.List;

@Disabled
@Autonomous(name="Red Close Lines", group="Auto")
public class RedCloseLineTest extends LinearOpModeEx {

    private List<LynxModule> allHubs;
    private AutonomousOpMode autonomousOpMode = null;

    @Override
    public void runOpMode() throws InterruptedException {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        autonomousOpMode = new RedCloseLines(hardwareMap, telemetry);
        autonomousOpMode.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
