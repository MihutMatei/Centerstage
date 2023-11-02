package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="test", group="Linear OpMode")

public class boboci extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    private enum runtype{
        NORMAL,
        DIFERIT
    }
    runtype currentruntype = runtype.NORMAL;


    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            switch (currentruntype){
                case NORMAL:{
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if(gamepad1.dpad_up){
                        leftFront.setPower(0.7);
                        rightFront.setPower(0.7);
                        leftRear.setPower(0.7);
                        rightRear.setPower(0.7);
                    }
                    else if(gamepad1.dpad_down){
                        leftFront.setPower(-0.7);
                        rightFront.setPower(-0.7);
                        leftRear.setPower(-0.7);
                        rightRear.setPower(-0.7);
                    }
                    else if(gamepad1.right_bumper){
                        leftFront.setPower(0.7);
                        rightFront.setPower(-0.7);
                        leftRear.setPower(0.7);
                        rightRear.setPower(-0.7);
                    }
                    else if(gamepad1.left_bumper){
                        leftFront.setPower(-0.7);
                        rightFront.setPower(0.7);
                        leftRear.setPower(-0.7);
                        rightRear.setPower(0.7);
                    }
                    else if(gamepad1.dpad_left){
                        leftFront.setPower(-0.7);
                        rightFront.setPower(0.7);
                        leftRear.setPower(0.7);
                        rightRear.setPower(-0.7);
                    }
                    else if(gamepad1.dpad_right){
                        leftFront.setPower(0.7);
                        rightFront.setPower(-0.7);
                        leftRear.setPower(-0.7);
                        rightRear.setPower(0.7);
                    }
                    else{
                        leftFront.setPower(0);
                        rightFront.setPower(0);
                        leftRear.setPower(0);
                        rightRear.setPower(0);
                    }
                }

                if(gamepad1.x)
                {
                    currentruntype = runtype.DIFERIT;
                }
                case DIFERIT:{
                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    if(gamepad1.y){
                        leftFront.setTargetPosition(1000);
                        leftRear.setTargetPosition(1000);
                        rightFront.setTargetPosition(1000);
                        rightRear.setTargetPosition(1000);

                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftFront.setPower(0.7);
                        rightFront.setPower(0.7);
                        leftRear.setPower(0.7);
                        rightRear.setPower(0.7);


                    }

                    if(gamepad1.x)
                    {
                        currentruntype = runtype.NORMAL;
                    }
                }
            }

        }


    }
}

