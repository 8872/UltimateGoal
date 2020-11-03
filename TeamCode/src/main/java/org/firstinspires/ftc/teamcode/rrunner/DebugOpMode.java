package org.firstinspires.ftc.teamcode.rrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.testchassis.rrunner.SampleMecanumDrive;

@Autonomous(group = "drive")
public class DebugOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            System.out.println(); // insert breakpoint here
        }
    }

}
