/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="OpMode")

public class Teleop extends OpMode {
    final int ARM_EXTEND_POSITION = 550;
    final int ARM_RETRACT_POSITION = 50;
    final double INTAKE_SERVO_OUT_TIME = 1000;

    public enum IntakeState {
        ARM_START,
        ARM_EXTEND,
        SERVO_IN,
        ARM_RETRACT,
        SERVO_OUT
    }
    public IntakeState intakeState = IntakeState.ARM_START;

    final int SLIDES_EXTEND_POSITION = 3000;
    final int SLIDES_RETRACT_POSITION = 0;
    final double OUTTAKE_SERVO_OUT_POSITION = 180;
    final double OUTTAKE_SERVO_IN_POSITION = 0;
    final double OUTTAKE_SERVO_OUT_TIME = 1000;
    final double OUTTAKE_SERVO_IN_TIME = 1000;

    public enum OuttakeState {
        SLIDES_START,
        SLIDES_EXTEND,
        SERVO_OUT,
        SERVO_IN,
        SLIDES_RETRACT
    }
    public OuttakeState outtakeState = OuttakeState.SLIDES_START;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;

    private DcMotor leftArm;
    private DcMotor rightArm;
    private CRServo intakeServo;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo outtakeServo;

    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");

        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");
        outtakeServo = hardwareMap.get(Servo.class, "outtake_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

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
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);

        leftArm.setPower(0.3);
        rightArm.setPower(0.3);

        switch (intakeState) {
            case ARM_START:
                if (outtakeState.name().equals("SLIDES_START") && gamepad1.x) {
                    leftArm.setTargetPosition(ARM_EXTEND_POSITION);
                    rightArm.setTargetPosition(ARM_EXTEND_POSITION);
                    intakeState = IntakeState.ARM_EXTEND;
                }
                break;
            case ARM_EXTEND:
                if (Math.abs(leftArm.getCurrentPosition() - ARM_EXTEND_POSITION) < 10) {
                    intakeServo.setPower(1.0);
                    intakeState = IntakeState.SERVO_IN;
                }
                break;
            case SERVO_IN:
                if (gamepad1.x) {
                    intakeServo.setPower(0.0);
                    leftArm.setTargetPosition(ARM_RETRACT_POSITION);
                    rightArm.setTargetPosition(ARM_RETRACT_POSITION);
                    intakeState = IntakeState.ARM_RETRACT;
                }
                break;
            case ARM_RETRACT:
                if (Math.abs(leftArm.getCurrentPosition() - ARM_RETRACT_POSITION) < 10) {
                    intakeServo.setPower(-1.0);
                    intakeTimer.reset();
                    intakeState = IntakeState.SERVO_OUT;
                }
                break;
            case SERVO_OUT:
                if (intakeTimer.milliseconds() >= INTAKE_SERVO_OUT_TIME) {
                    intakeServo.setPower(0.0);
                    intakeState = IntakeState.ARM_START;
                }
                break;
            default:
                intakeState = IntakeState.ARM_START;
        }

        leftSlide.setPower(0.5);
        rightSlide.setPower(0.5);

        switch (outtakeState) {
            case SLIDES_START:
                if (intakeState.name().equals("ARM_START") && gamepad1.y) {
                    leftSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                    rightSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                    outtakeState = OuttakeState.SLIDES_EXTEND;
                }
                break;
            case SLIDES_EXTEND:
                if (Math.abs(leftSlide.getCurrentPosition() - SLIDES_EXTEND_POSITION) < 10 && gamepad1.y) {
                    outtakeServo.setPosition(OUTTAKE_SERVO_OUT_POSITION);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.SERVO_OUT;
                }
                break;
            case SERVO_OUT:
                if (outtakeTimer.milliseconds() >= OUTTAKE_SERVO_OUT_TIME) {
                    outtakeServo.setPosition(OUTTAKE_SERVO_IN_POSITION);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.SERVO_IN;
                }
                break;
            case SERVO_IN:
                if (outtakeTimer.milliseconds() >= OUTTAKE_SERVO_IN_TIME){
                    leftSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    rightSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    outtakeState = OuttakeState.SLIDES_RETRACT;
                }
                break;
            case SLIDES_RETRACT:
                if (Math.abs(leftSlide.getCurrentPosition() - SLIDES_RETRACT_POSITION) < 10 && gamepad1.y) {
                    outtakeState = outtakeState.SLIDES_START;
                }
                break;
            default:
                outtakeState = outtakeState.SLIDES_START;
        }

        telemetry.addData("Status", "Runtime: " + runtime.toString());
        telemetry.update();
    }
}