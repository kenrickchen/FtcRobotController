package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.Constants.*;

public class LeftAuto extends LinearOpMode {
    private DcMotor leftArm;
    private DcMotor rightArm;
    private CRServo intakeServo;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo outtakeServo;

    private final Pose2d START_POSE = new Pose2d(-23, -61, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");
        outtakeServo = hardwareMap.get(Servo.class, "outtake_servo");

        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setTargetPosition(ARM_RETRACT_POSITION);
        rightArm.setTargetPosition(ARM_RETRACT_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeServo.setDirection(CRServo.Direction.REVERSE);
        intakeServo.setPower(0.0);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
        rightSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeServo.setPosition(OUTTAKE_SERVO_IN_POSITION);

        telemetry.addData("Status", "Initialized");

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(START_POSE)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    leftSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                    rightSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                })
                .turn(Math.toRadians(35))
                .addTemporalMarker(() -> {
                    outtakeServo.setPosition(OUTTAKE_SERVO_OUT_POSITION);
                })
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    leftSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    rightSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    leftArm.setTargetPosition(ARM_EXTEND_POSITION);
                    rightArm.setTargetPosition(ARM_EXTEND_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intakeServo.setPower(1.0);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    leftArm.setTargetPosition(ARM_RETRACT_POSITION);
                    rightArm.setTargetPosition(ARM_RETRACT_POSITION);
                })
                .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intakeServo.setPower(1.0);
                })
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
                .build();
    }
}
