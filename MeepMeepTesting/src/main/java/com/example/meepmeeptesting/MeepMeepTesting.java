package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity testpreload_and_backboard = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, Math.toRadians(80), Math.toRadians(80), 13.61)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11,-60,Math.toRadians(90)))//0

//                        .splineToLinearHeading(new Pose2d(11,-35,Math.toRadians(90)),Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(8,-32,Math.toRadians(155)),Math.toRadians(110))
                        .splineToLinearHeading(new Pose2d(16,-34,Math.toRadians(15)),Math.toRadians(75))
                                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35,-55,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(45,-42,Math.toRadians(0)))

                        .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_LIGHT)

                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(testpreload_and_backboard)

                .start();

    }

}

