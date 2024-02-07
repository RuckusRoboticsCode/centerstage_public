package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.blue.BlueCloseLines;
import org.firstinspires.ftc.teamcode.auto.red.RedCloseAuto;
import org.firstinspires.ftc.teamcode.auto.util.AutonomousOpMode;

//@Photon
@Autonomous(name="Auto Test", group="Auto")
public class testAuto extends LinearOpMode {

    AutonomousOpMode autonomousOpMode;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        autonomousOpMode = new BlueCloseLines(hardwareMap, telemetry);
        autonomousOpMode.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            autonomousOpMode.runLoop();
            telemetry.update();
        }
    }
}
