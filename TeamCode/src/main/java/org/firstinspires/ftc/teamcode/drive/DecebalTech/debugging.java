package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="debugging", group="Linear Opmode")

@Config


public class debugging extends LinearOpMode {

    private DcMotor start;

    @Override
    public void runOpMode() throws InterruptedException {

        start = hardwareMap.get(DcMotor.class,"1");
        waitForStart();


        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {
            start.setPower(1);
            telemetry.addLine("what");
            telemetry.update();
        }
    }


}