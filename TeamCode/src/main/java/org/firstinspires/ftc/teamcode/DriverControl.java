package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Base.RobotStruct;
// Game pad routing docs for reference: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

@TeleOp(name="DriverControl")
public class DriverControl extends RobotStruct {
    ElapsedTime nap = new ElapsedTime();

    public enum STATE {
        EXIT,
        // sequentially move to leave pit
        FORWARD_TO_LEAVE_PIT,
        TRANSLATE_RIGHT,
        POSITION_TO_ZERO,
        MOVE_FORWARD_TO_GOAL,
        // move arm and drop block
        MOVE_ARM_DOWN,
        RELEASE_BLOCK,
        MOVE_ARM_UP,
        // sequentially move back to pit
        MOVE_BACK_TO_WALL,
        TURN_LEFT,
        TRANSLATE_LEFT,
        MOVE_BACK_TO_PIT
    }

    public STATE state = STATE.FORWARD_TO_LEAVE_PIT;

    public void init() {
        nap.reset();
    }

    @Override
    public void loop() {
//        double rotate = gamepad1.left_stick_x;
        double armSpeed = 0.75 * gamepad2.left_stick_y;
        double duckDropperSpeed = 5 * (gamepad2.right_trigger - gamepad2.left_trigger);

        setDuckDropperSpeed(duckDropperSpeed);
        setArmSpeed(-armSpeed);
        initDriver();


        while (gamepad1.dpad_down){
            setDriverMotorPower(0.25,0.25,0.25, 0.25);

            if (!gamepad1.dpad_down){
                setDriverPowerZERO();
            }
        }

        while (gamepad1.dpad_up){
            setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);

            if (!gamepad1.dpad_up){
                setDriverPowerZERO();
            }
        }

        while (gamepad1.dpad_right){
            translateRight(0.25);

            if (!gamepad1.dpad_right){
                translateRight(0);
            }
        }

        while (gamepad1.dpad_left){
            translateLeft(0.25);

            if (!gamepad1.dpad_left){
                translateLeft(0);
            }
        }

        if (!gamepad2.dpad_up){
            setIntakeSpeed(0);
        }

        if (!gamepad2.dpad_down){
            setIntakeSpeed(0);
        }

        while (gamepad2.dpad_up){
            setIntakeSpeed(0.5);

            if (!gamepad2.dpad_up){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_down){
            setIntakeSpeed(-0.5);

            if (!gamepad2.dpad_down){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_up && gamepad2.y){
            setIntakeSpeed(0.5);

            if (!gamepad2.dpad_up){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_down && gamepad2.y){
            setIntakeSpeed(-0.5);

            if (!gamepad2.dpad_down) {
                setIntakeSpeed(0);
            }
        }

//        close
        if(gamepad2.a) {
            setClawPos(0.91, 0.09);
        }

//        open
        if(gamepad2.b) {
            setClawPos(0.85, 0.15);
        }

        if (gamepad2.x) {
            setHoldPosition(0);

        }

        if (gamepad2.y) {
            setPushPosition(0.1);
        }

        // completely experimental and has not been tested: autonomous-like sequence
        if (gamepad1.a) {
            switch (state) {
                case FORWARD_TO_LEAVE_PIT:
                    nap.reset();
                    // custom sleep
                    while (nap.time() < 1100) {
                        setDriverMotorPower(0.25, 0.25, 0.25, 0.25);
                    }
                    setDriverMotorPower(0,0,0,0);
                    state = STATE.TRANSLATE_RIGHT;
                    break;

                case TRANSLATE_RIGHT:
                    nap.reset();
                    // translate right
                    while (nap.time() < 900) {
                        setDriverMotorPower(-0.20, 0.20, 0.20, -0.20);
                    }
                    setDriverMotorPower(0,0,0,0);
                    state = STATE.POSITION_TO_ZERO;
                    break;

                // TODO: CHECK - IS THIS AN INFINITE LOOP?
                case POSITION_TO_ZERO:
                    // adjust bot so heading is approx. zero
                    double currentPosition = getAngle();
                    while (currentPosition < -1.0 || currentPosition > 1.0) {
                        // self centering
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        if (currentPosition > 1.0) {
                            telemetry.addData("heading", currentPosition);
                            telemetry.update();
                            setDriverMotorPower(0.15, -0.15, 0.15, -0.15);
                            currentPosition = getAngle();
                        }

                        if (currentPosition < -1.0) {
                            telemetry.addData("heading", currentPosition);
                            telemetry.update();
                            setDriverMotorPower(-0.15, 0.15, -0.15, 0.15);
                            currentPosition = getAngle();
                        }
                    }
                    setDriverMotorPower(0,0,0,0);
                    state = STATE.MOVE_FORWARD_TO_GOAL;
                    break;

                case MOVE_FORWARD_TO_GOAL:
                    nap.reset();
                    while (nap.time() < 1000) {
                        setDriverMotorPower(0.50, 0.50, 0.50, 0.50);
                    }
                    setDriverMotorPower(0,0,0,0);
                    state = STATE.EXIT;

                case EXIT:
                    setDriverMotorPower(0,0,0,0);
            }
        }
    }
}