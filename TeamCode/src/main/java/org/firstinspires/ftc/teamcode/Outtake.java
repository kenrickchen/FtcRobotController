package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake extends OpMode {
    final int SLIDES_EXTEND_POSITION = 5000;
    final int SLIDES_RETRACT_POSITION = 0;
    final double SERVO_OUT_POSITION = 180;
    final double SERVO_IN_POSITION = 0;
    final double SERVO_OUT_TIME = 1000;
    final double SERVO_IN_TIME = 1000;

    public enum State {
        SLIDES_START,
        SLIDES_EXTEND,
        SERVO_OUT,
        SERVO_IN,
        SLIDES_RETRACT
    }
    public State state = State.SLIDES_START;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo servo;
    private ElapsedTime timer = new ElapsedTime();

    public Outtake(DcMotor leftSlide, DcMotor rightSlide, Servo servo) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.servo = servo;
    }

    @Override
    public void init() {
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
        rightSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo.setPosition(SERVO_IN_POSITION);
    }

    @Override
    public void loop() {
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        switch (state) {
            case SLIDES_START:
                if (Teleop.intake.state.name().equals("ARM_START") && gamepad1.y) {
                    leftSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                    rightSlide.setTargetPosition(SLIDES_EXTEND_POSITION);
                    state = State.SLIDES_EXTEND;
                }
                break;
            case SLIDES_EXTEND:
                if (Math.abs(leftSlide.getCurrentPosition() - SLIDES_EXTEND_POSITION) < 10 && gamepad1.y) {
                    servo.setPosition(SERVO_OUT_POSITION);
                    timer.reset();
                    state = State.SERVO_OUT;
                }
                break;
            case SERVO_OUT:
                if (timer.milliseconds() >= SERVO_OUT_TIME) {
                    servo.setPosition(SERVO_IN_POSITION);
                    timer.reset();
                    state = State.SERVO_IN;
                }
                break;
            case SERVO_IN:
                if (timer.milliseconds() >= SERVO_IN_TIME){
                    leftSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    rightSlide.setTargetPosition(SLIDES_RETRACT_POSITION);
                    state = State.SLIDES_RETRACT;
                }
                break;
            case SLIDES_RETRACT:
                if (Math.abs(leftSlide.getCurrentPosition() - SLIDES_RETRACT_POSITION) < 10 && gamepad1.y) {
                    state = State.SLIDES_START;
                }
                break;
            default:
                state = State.SLIDES_START;
        }
    }
}
