package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="Drive", group="Linear Opmode")
@Config


public class Drive extends LinearOpMode {

    private RobotUtils robot;
    enum ChasisState{
        DRIVE,
        TURBO,
        PRECISION
    }
    enum SliderState{
        MANUAL,
        AUTO
    }
    private SliderState sliderstate = SliderState.AUTO;
    private ChasisState chasisState = ChasisState.DRIVE;

    private SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotUtils(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.cuva_return();
        robot.arm_return();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            /*-------------P2 CONTROLS-------------------------------------*/

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
//            if(gamepad1.triangle) robot.drone_launch();
//            if(gamepad1.start) robot.drone_reset();
            if(gamepad1.dpad_up) robot.intake_on();
            if(gamepad1.dpad_down) robot.intake_off();
            if(gamepad1.dpad_left) robot.intake_reverse();

            /*-------------P2 CONTROLS-------------------------------------*/

            switch (sliderstate){
                case AUTO:
                    if(gamepad2.dpad_down) robot.go_sliders_down();
                    if(gamepad2.dpad_up) robot.go_sliders_high();
                    if(gamepad2.dpad_left) robot.go_sliders_mid();
                    if(gamepad2.dpad_right) robot.go_sliders_low();
                    if(gamepad2.triangle){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sliderstate = SliderState.MANUAL;
                    }
                    if (gamepad2.circle){
                        robot.cuva_return();
                        robot.arm_return();

                    }
                    if(gamepad2.cross){
                        robot.cuva_score();
                        robot.arm_extend();
                    }
                    if(gamepad2.square) robot.pixel_drop_one();
                    break;
                case MANUAL:
                    if(gamepad2.left_stick_y>0.3){
                        robot.sliderLeft.setPower(gamepad2.left_stick_y);
                        robot.sliderRight.setPower(-gamepad2.left_stick_y);
                    }
                   else if(gamepad2.left_stick_y<-0.3){
                        robot.sliderLeft.setPower(gamepad2.left_stick_y);
                        robot.sliderRight.setPower(-gamepad2.left_stick_y);
                   }
                   else{
                        robot.sliderLeft.setPower(-0);
                        robot.sliderRight.setPower(0);
                    }
                    if(gamepad2.triangle){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        //teoretic cele 2 lini de dedesubt sunt redundante petnru ca sunt
                        //prezente si in RobotUtils in toate functiile de go to position
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderstate = SliderState.AUTO;
                    }
                    if(gamepad2.right_stick_button){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        //teoretic cele 2 lini de dedesubt sunt redundante petnru ca sunt
                        //prezente si in RobotUtils in toate functiile de go to position
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderstate = SliderState.AUTO;
                    }
                    if (gamepad2.circle){
                        robot.cuva_return();
                        robot.arm_return();

                    }
                    if(gamepad2.cross){
                        robot.cuva_score();
                        robot.arm_extend();
                    }
                    if(gamepad2.square) robot.pixel_drop_one();
                    break;
            }


//            if(robot.sliderRight.getCurrentPosition()>0 && robot.sliderRight.getCurrentPosition()<50) {
//                    robot.cuva_return();
//                    robot.arm_return();
//
//            }
//            if(robot.sliderRight.getCurrentPosition()>50 && robot.sliderRight.getCurrentPosition()<1200) {
//
//                    robot.cuva_clearance();
//                    robot.arm_clearance();
//
//            }
//            if(robot.sliderRight.getCurrentPosition()>1400 && robot.sliderRight.getCurrentPosition()<3000) {
//
//                    robot.cuva_score();
//                    robot.arm_extend();
//
//            }

            telemetry.addLine(sliderstate.toString());

            drive.update();
            telemetry.update();
        }
    }




}