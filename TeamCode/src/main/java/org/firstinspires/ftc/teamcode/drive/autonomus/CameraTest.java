package org.firstinspires.ftc.teamcode.drive.autonomus;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous(name = "CameraTest")
public class CameraTest extends LinearOpMode {
    OpenCvCamera webcam;
    DetectionPipeline detectionPipeline;
    boolean bCameraOpened = false;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        detectionPipeline.setGridSize(2);

        double left_avg,right_avg;

        int zone = 0;
        sleep(5000);
        while (!opModeIsActive() && !isStopRequested()) {

            left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
            right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;

            if (left_avg <= 123.5)
                zone = 1;
            else if (right_avg <= 123.5)
                zone = 2;
            else
                zone = 3;

            telemetry.addData("Zone", zone);
            telemetry.addData("Left", left_avg);
            telemetry.addData("Right", right_avg);

            telemetry.update();
        }


        if (!opModeIsActive()) return;

    }
}