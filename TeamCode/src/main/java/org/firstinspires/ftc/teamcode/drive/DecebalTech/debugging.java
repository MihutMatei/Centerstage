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
import org.firstinspires.ftc.teamcode.util.RobotUtilsDebug;

@TeleOp(name="debugging", group="Linear Opmode")
@Config
public class debugging extends LinearOpMode {


    private RobotUtilsDebug robot = new RobotUtilsDebug(hardwareMap);
    public double pos1=0;
    public double pos2=0;
    public double pos3=0;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();


        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad1.circle){
                robot.axon_arm_left.setPosition(pos1);
//                robot.axon_arm_right.setPosition(pos1);
            }
            if(gamepad1.square){
//                robot.axon_arm_left.setPosition(pos2);
                robot.axon_arm_right.setPosition(pos2);
            }
            if(gamepad1.cross){
                robot.axon_rotire_cuva.setPosition(pos3);
            }


            telemetry.addData("axon stanga pos: ",robot.axon_arm_left.getPosition());
            telemetry.addData("axon dreapta pos: ",robot.axon_arm_right.getPosition());
            telemetry.addData("axon rotire cuva pos: ",robot.axon_rotire_cuva.getPosition());
//            telemetry.addData("pozitie servo pixel ",robot.pixel_servo.getPosition());
//            telemetry.addData("slider stanga pos",robot.sliderLeft.getCurrentPosition());
//            telemetry.addData("slider dreapta pos",robot.sliderRight.getCurrentPosition());
            telemetry.update();
        }

    }


}