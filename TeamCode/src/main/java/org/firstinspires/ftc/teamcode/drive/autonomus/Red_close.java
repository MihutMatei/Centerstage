package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipelineMatei;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red_Close", group="AUTINOMOUSGOOD")
@Config


public class Red_close extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;
    DetectionPipelineMatei detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;
    private double loopTime=0,loop;

    private  boolean nu_stiu_sa_codez2 = true;

    enum ZoneType{
        RIGHT,
        LEFT,
        CENTER
    }
    ZoneType zone =  ZoneType.CENTER;
    ZoneType zoneFinal =  ZoneType.CENTER;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipeline = new DetectionPipelineMatei();
        webcam.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(10);
        drive =new SampleMecanumDrive(hardwareMap);
        robot = new RobotUtils(hardwareMap);
        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(2000);
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
//        sleep(2000);
        Pose2d startPose = new Pose2d(11,-60,Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence pune_preload_stanga = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(7,-31,Math.toRadians(155)),Math.toRadians(110))
                .build();
        TrajectorySequence pune_preload_dreapta = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12.5,-31,Math.toRadians(15)),Math.toRadians(75))
                .build();
        TrajectorySequence pune_preload_mijloc = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(11,-30,Math.toRadians(90)),Math.toRadians(90))
                .build();

        TrajectorySequence alignst = drive.trajectorySequenceBuilder(pune_preload_stanga.end())
                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(155)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.go_sliders_low_auto();
                })

                .build();
        TrajectorySequence align3 = drive.trajectorySequenceBuilder(pune_preload_dreapta.end())
                .lineToLinearHeading(new Pose2d(9,-55,Math.toRadians(15)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.go_sliders_low_auto();
                })

                .build();
        TrajectorySequence align2 = drive.trajectorySequenceBuilder(pune_preload_mijloc.end())
                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.go_sliders_low_auto();
                })


                .build();
        TrajectorySequence score_preload_zone_left_u = drive.trajectorySequenceBuilder(alignst.end())
                .lineToLinearHeading(new Pose2d(50,-26.5,Math.toRadians(-1)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.arm_extend();
                })
                .addTemporalMarker(0.25,()->{
                    robot.cuva_score();

                })
                .build();
        TrajectorySequence score_preload_zone_mid_m = drive.trajectorySequenceBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(50,-31,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.arm_extend();
                })
                .addTemporalMarker(0.25,()->{
                    robot.cuva_score();

                })

                .build();
        TrajectorySequence score_preload_zone_right_d = drive.trajectorySequenceBuilder(align3.end())
                .lineToLinearHeading(new Pose2d(50,-36,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.arm_extend();
                })
                .addTemporalMarker(0.25,()->{
                    robot.cuva_score();

                })
                .build();
        TrajectorySequence prepark = drive.trajectorySequenceBuilder(score_preload_zone_mid_m.end())
                .lineToLinearHeading(new Pose2d(36,-30,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(prepark.end())
                .lineToLinearHeading(new Pose2d(52,-58,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();



        while (!isStarted() && !isStopRequested()) {

            double zoneleft = detectionPipeline.getZoneLuminosity(4);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(64)
                                                        ,detectionPipeline.getZoneLuminosity(54))
                                                        ,detectionPipeline.getZoneLuminosity(74))
                                                        ,detectionPipeline.getZoneLuminosity(44));


            if (zoneleft<zonemid && zoneleft<95) zone = ZoneType.LEFT;
            else if (zonemid < zoneleft && zonemid<95)zone = ZoneType.CENTER;
            else zone = ZoneType.RIGHT;


            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone left",zoneleft);
            telemetry.addData("luminosity zone mid",zonemid);
            loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
//            telemetry.addData("luminosity zone 6",detectionPipeline.getZoneLuminosity(8));

            telemetry.update();

            robot.arm_return();
            robot.cuva_return();

        }
        bCameraOpened = false;
        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
        }
        switch(zoneFinal){
            case LEFT:
                drive.followTrajectorySequence(pune_preload_stanga);
                sleep(300);

                drive.followTrajectorySequence(alignst);
                sleep(1000);

                drive.followTrajectorySequence(score_preload_zone_left_u);
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.arm_return();
                sleep(100);
                robot.cuva_return();
                sleep(500);
                robot.go_sliders_down();
                sleep(1000);
                drive.followTrajectorySequence(park);
                break;
            case CENTER:
                drive.followTrajectorySequence(pune_preload_mijloc);
                sleep(300);

                drive.followTrajectorySequence(align2);
                sleep(1000);

                drive.followTrajectorySequence(score_preload_zone_mid_m);
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.arm_return();
                sleep(100);
                robot.cuva_return();
                sleep(500);
                robot.go_sliders_down();
                sleep(1000);
                drive.followTrajectorySequence(park);

                break;
            case RIGHT:
                drive.followTrajectorySequence(pune_preload_dreapta);
                sleep(300);
                drive.followTrajectorySequence(align3);
                sleep(1000);
                drive.followTrajectorySequence(score_preload_zone_right_d);
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                robot.pixel_drop_one();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.arm_return();
                sleep(100);
                robot.cuva_return();
                sleep(500);
                robot.go_sliders_down();
                sleep(1000);
                drive.followTrajectorySequence(park);

                break;
        }
        if (!opModeIsActive()) return;




    }
}