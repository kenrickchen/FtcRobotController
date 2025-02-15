package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LeftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-23, -61, Math.toRadians(90));


        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
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
    }
}
