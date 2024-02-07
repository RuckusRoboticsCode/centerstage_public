package org.firstinspires.ftc.teamcode.auto.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.util.CustomTypes;
import org.firstinspires.ftc.teamcode.auto.util.VisionController;

@Disabled
@Autonomous(name="Right Vision Test")
public class RightVisionTest extends LinearOpMode {

    VisionController visionController2;
    FtcDashboard ftcDashboard;
    CustomTypes.PropLocation propLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        visionController2 = new VisionController(hardwareMap, false, true, true, telemetry, ftcDashboard);

        sleep(2000);

        while (opModeInInit() && !isStopRequested()) {
//            propLocation = visionController2.getLocationOnce();
            visionController2.updateTelemetry();
//            telemetry.addData("Prop Location", propLocation.getLocation());
            telemetry.update();
        }
    }
}
