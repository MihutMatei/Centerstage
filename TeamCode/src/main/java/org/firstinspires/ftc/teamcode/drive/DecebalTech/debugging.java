package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="debugging", group="Linear Opmode")
@Config
public class debugging extends LinearOpMode {

    enum ChasisState{
        DRIVE,
        TURBO,
        PRECISION
    }
    SampleMecanumDrive drive;
    ChasisState chasisState =ChasisState.DRIVE;
    private RobotUtils robot;
    public static double pos1=0.8;
    public static double pos2=0.52;
    public static double pos3=0.05;
    public static double pos4=1;

    public static double pizel_1=0;
    public static double pizel_2=0;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotUtils(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
                        chasisState = ChasisState.PRECISION;
                    }
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
//            if(gamepad1.circle){
//                robot.axon_arm_left.setPosition(pos2    );
//                robot.axon_arm_right.setPosition(pos2);
////                robot.axon_rotire_cuva.setPosition(pos2);
//            }
//            if(gamepad1.square){
//                robot.axon_arm_left.setPosition(pos1);
//                robot.axon_arm_right.setPosition(pos1);
////                robot.axon_rotire_cuva.setPosition(pos1);
//            }
//            if(gamepad1.triangle){
//                robot.axon_rotire_cuva.setPosition(pos3);
//            }
//            if(gamepad1.cross) {
//                robot.axon_rotire_cuva.setPosition(pos4);
//            }
//            if(gamepad1.dpad_down){
//                robot.pixel_servo.setPosition(pizel_1);
//            }
//            if(gamepad1.dpad_up){
//                robot.pixel_servo.setPosition(pizel_2);
//            }
//            if(gamepad2.x){
//                robot.motorIntake.setPower(0.5);
//            }
//            if(gamepad2.a){
//                robot.motorIntake.setPower(-0.5);
//            }
//            if(gamepad2.b){
//                robot.motorIntake.setPower(0);
//            }
                if(gamepad1.a) robot.pixel_drop_one();

                if(gamepad1.b) {
                    robot.go_sliders_high();
                    robot.pixel_servo.setPosition(pos2);
                }
                if(gamepad1.y) robot.go_sliders_down();

                if(gamepad1.dpad_up) robot.intake_on();
                if(gamepad1.dpad_down) robot.intake_off();

                if(gamepad1.dpad_left) robot.arm_extend();
                if(gamepad1.dpad_right) robot.arm_return();

                if(gamepad1.right_stick_button) robot.cuva_score();
                if(gamepad1.left_stick_button) robot.cuva_return();

                if(gamepad2.square) {
                    robot.drone_launch();
                }
                if(gamepad2.cross){
                    robot.drone_reset();
                }


            telemetry.addData("axon stanga pos:  ",robot.axon_arm_left.getPosition());
            telemetry.addData("axon dreapta pos: ",robot.axon_arm_right.getPosition());
            telemetry.addData("axon rotire cuva pos: ",robot.axon_rotire_cuva.getPosition());
            telemetry.addData("pozitie servo pixel ",robot.pixel_servo.getPosition());
            telemetry.addData("slider stanga pos",robot.sliderLeft.getCurrentPosition());
            telemetry.addData("slider dreapta pos",robot.sliderRight.getCurrentPosition());
            telemetry.update();
        }

    }


}