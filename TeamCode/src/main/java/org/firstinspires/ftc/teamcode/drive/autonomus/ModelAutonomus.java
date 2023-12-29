package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipelineMatei;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="ModelAutonomus", group="AUTINOMOUSGOOD")
@Config


public class ModelAutonomus extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;
    DetectionPipelineMatei detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;

    enum ZoneType{
        RIGHT,
        LEFT,
        CENTER
    }
    ZoneType zone =  ZoneType.CENTER;
    String Pozitie_Preload;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Pose2d startPose = new Pose2d(11,-60,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        detectionPipeline = new DetectionPipelineMatei();
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

        TrajectorySequence pune_preload_stanga = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(8,-20,Math.toRadians(155)),Math.toRadians(110))
                .build();
        TrajectorySequence pune_preload_dreapta = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(16,-34,Math.toRadians(15)),Math.toRadians(75))
                .build();
        TrajectorySequence pune_preload_mijloc = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(11,-35,Math.toRadians(90)),Math.toRadians(90))
                .build();




        sleep(2000);
        detectionPipeline.setGridSize(3);

        while (!isStarted() && !isStopRequested()) {
//            double left_mean = (detectionPipeline.getZoneLuminosity(1)+detectionPipeline.getZoneLuminosity(2)+detectionPipeline.getZoneLuminosity(3))/3.0;
//            double center_mean = (detectionPipeline.getZoneLuminosity(4)+detectionPipeline.getZoneLuminosity(5)+detectionPipeline.getZoneLuminosity(6))/3.0;
//            double right_mean = (detectionPipeline.getZoneLuminosity(7)+detectionPipeline.getZoneLuminosity(8)+detectionPipeline.getZoneLuminosity(9))/3.0;
            //TODO: codul comentat verifica doar zonele de pe mijlocul camerei, codul activ verifica valorile medii;
            if(detectionPipeline.getZoneLuminosity(2) <130)
               zone=ZoneType.LEFT;
            else if(detectionPipeline.getZoneLuminosity(5) <130 )
               zone=ZoneType.CENTER;
            else zone=ZoneType.RIGHT;
//            if (left_mean>center_mean && left_mean>right_mean) zone = ZoneType.LEFT;
//            else if (right_mean>center_mean && right_mean>left_mean) zone = ZoneType.RIGHT;
//            else if (center_mean>left_mean &&center_mean > right_mean) zone = ZoneType.CENTER;

            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone 2",detectionPipeline.getZoneLuminosity(2));
            telemetry.addData("luminosity zone 5",detectionPipeline.getZoneLuminosity(5));
            telemetry.addData("luminosity zone 6",detectionPipeline.getZoneLuminosity(8));
//            telemetry.addData("left_mean ",left_mean);
//            telemetry.addData("center_mean",center_mean);
//            telemetry.addData("right_mean",right_mean);
            telemetry.update();




        }
//        if(zone==ZoneType.LEFT) drive.followTrajectorySequence(pune_preload_stanga);
//        else if(zone==ZoneType.CENTER) drive.followTrajectorySequence(pune_preload_mijloc);
//        else drive.followTrajectorySequence(pune_preload_dreapta);
        TrajectorySequence re = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(11 ,-38))
                .lineToLinearHeading(new Pose2d(11,-50))
                .build();
        sleep(300);
        drive.followTrajectorySequence(re);
        if (!opModeIsActive()) return;



        telemetry.update();

    }
}