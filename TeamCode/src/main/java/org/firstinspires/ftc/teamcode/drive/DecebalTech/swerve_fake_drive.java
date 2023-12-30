package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SwerveUtils;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name ="swerve_fake",group = "drive")
public class swerve_fake_drive extends LinearOpMode {

    private SwerveUtils robot;
    private double loopTime = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SwerveUtils(hardwareMap);
        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.right_stick_button) {
                robot.set_servos_power(0.5, 0.5, 0.5, 0.5);
            } else if (gamepad1.left_stick_button) {
                robot.set_servos_power(-0.5, -0.5, -0.5, -0.5);
            } else robot.set_servos_power(0.01, 0.01, 0.01, 0.01);

            if (gamepad1.left_stick_y > 0.2) {
                robot.set_motors_power(0.5, 0.5, 0.5, 0.5);
//                robot.set_servos_power(1, 1, 1, 1);
            } else if (gamepad1.left_stick_y < -0.2) {
                robot.set_motors_power(-0.5, -0.5, -0.5, -0.5);
            } else if (gamepad1.y) robot.set_motors_power(0.5, -0.5, 0.5, -0.5);
            else if (gamepad1.a) robot.set_motors_power(-0.5, 0.5, -0.5, 0.5);
            else robot.set_motors_power(0, 0, 0, 0);
        telemetry.addData("servo backright pow: ",robot.backRightServo.getPower());
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;

        telemetry.update();
        }

        

    }
}
