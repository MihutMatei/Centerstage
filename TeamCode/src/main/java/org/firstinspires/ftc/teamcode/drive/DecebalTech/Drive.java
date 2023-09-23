package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="Drive", group="Linear Opmode")
@Config


public class Drive extends LinearOpMode {

    
    enum ChasisState{
        DRIVE,
        TURBO,
        PRECISION
    }

    private ChasisState chasisState = ChasisState.DRIVE;

    private SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {

        PhotonCore.experimental.setSinglethreadedOptimized(true);
        PhotonCore.enable();


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {






            switch (chasisState){
                case DRIVE:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/1.5,
                                    -gamepad1.left_stick_x/1.5,
                                    -gamepad1.right_stick_x/1.5
                            )
                    );

                    if (gamepad1.right_trigger>0.3) {

                        chasisState = ChasisState.TURBO;
                    }
                    if (gamepad1.left_trigger>0.3) {

                        chasisState = ChasisState.PRECISION;                    }
                    break;
                case TURBO:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.right_trigger==0) {

                        chasisState = ChasisState.DRIVE;
                    }
                    break;
                case PRECISION:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/3,
                                    -gamepad1.left_stick_x/3,
                                    -gamepad1.right_stick_x/3
                            )
                    );

                    if (gamepad1.left_trigger==0) {

                        chasisState = ChasisState.DRIVE;
                    }

            }







            drive.update();

        }
    }




}