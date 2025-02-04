package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends OpMode {
    final int ARM_EXTEND_POSITION = 1000;
    final int ARM_RETRACT_POSITION = 0;
    final double SERVO_OUT_TIME = 1000;

    public enum State {
        ARM_START,
        ARM_EXTEND,
        SERVO_IN,
        ARM_RETRACT,
        SERVO_OUT
    }
    public State state = State.ARM_START;

    private DcMotor leftArm;
    private DcMotor rightArm;
    private CRServo servo;
    private ElapsedTime timer = new ElapsedTime();

    public Intake(DcMotor leftArm, DcMotor rightArm, CRServo crServo) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.servo = crServo;
    }

    @Override
    public void init() {
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setTargetPosition(ARM_RETRACT_POSITION);
        rightArm.setTargetPosition(ARM_RETRACT_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servo.setPower(0.0);
    }

    @Override
    public void loop() {
        leftArm.setPower(1.0);
        rightArm.setPower(1.0);

        switch (state) {
            case ARM_START:
                if (Teleop.outtake.state.name().equals("SLIDES_START") && gamepad1.x) {
                    leftArm.setTargetPosition(ARM_EXTEND_POSITION);
                    rightArm.setTargetPosition(ARM_EXTEND_POSITION);
                    state = State.ARM_EXTEND;
                }
                break;
            case ARM_EXTEND:
                if (Math.abs(leftArm.getCurrentPosition() - ARM_EXTEND_POSITION) < 10) {
                    servo.setPower(1.0);
                    state = State.SERVO_IN;
                }
                break;
            case SERVO_IN:
                if (gamepad1.x) {
                    servo.setPower(0.0);
                    leftArm.setTargetPosition(ARM_RETRACT_POSITION);
                    rightArm.setTargetPosition(ARM_RETRACT_POSITION);
                    state = State.ARM_RETRACT;
                }
                break;
            case ARM_RETRACT:
                if (Math.abs(leftArm.getCurrentPosition() - ARM_RETRACT_POSITION) < 10) {
                    servo.setPower(-1.0);
                    timer.reset();
                    state = State.SERVO_OUT;
                }
                break;
            case SERVO_OUT:
                if (timer.milliseconds() >= SERVO_OUT_TIME) {
                    servo.setPower(0.0);
                    state = State.ARM_START;
                }
                break;
            default:
                state = State.ARM_START;
        }
    }
}
