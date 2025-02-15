package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-23, -61, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(225))
                        .waitSeconds(1)
                        .turn(Math.toRadians(35))
                        .waitSeconds(1)
                        .forward(7)
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(1)
                        .turn(Math.toRadians(57))
                        .forward(7)
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-45, -24, Math.toRadians(180)))
                        .waitSeconds(1)
                        .forward(5)
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-30, -15), Math.toRadians(0))
                        .waitSeconds(1)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}