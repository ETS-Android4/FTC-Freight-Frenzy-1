package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoRobotStruct extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo servoClaw1;
    private Servo servoClaw2;
    private Servo servoPush;
    private Servo servoHold;
    private DcMotor motorDuckDropper;
    private DcMotorEx motorIntake;
    private DcMotorEx motorArm;
    private DcMotorEx motorArmDuo;
    private DistanceSensor distanceBack;
    private DistanceSensor distanceFront;
    // accessed in RedDuck.java directly without get/set methods therefore not private vars
    public ColorSensor colorSensor;
    public ColorSensor whiteLine;

    @Override
    public void runOpMode() throws InterruptedException { }

    public void initRunner() throws InternalError {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorDuckDropper = hardwareMap.get(DcMotorEx.class, "motor duck dropper");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw1 = hardwareMap.get(Servo.class, "servo claw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo claw2");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distance back");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distance front");
        motorIntake = hardwareMap.get(DcMotorEx.class, "motor intake");
        servoPush = hardwareMap.get(Servo.class, "servo Push");
        servoHold = hardwareMap.get(Servo.class, "servo Hold");
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");
        whiteLine = hardwareMap.get(ColorSensor.class, "whiteLineDetector");

        // reverse direction for this drive train
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getDistanceFront() {
        return distanceFront.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceBack() {
        return distanceBack.getDistance(DistanceUnit.INCH);
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower, int s) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
        sleep(s);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(100);
    }

    public void setDistanceAndMoveForward(double _front_distance){
        double distFront = getDistanceFront();
        telemetry.addData("Dist front ", distFront);

        while (distFront > _front_distance) {
            // telemetry.addData("Dist front ", distFront);
            // move forward
            setDriverMotorPower(0.25,0.25,0.25,0.25);
            distFront = getDistanceFront();
        }

        setDriverMotorPower(0.0,0.0,0.0,0.0, 10);
    }

    public void setDistanceAndMoveBackward(double back_distance){
        double distBack = getDistanceBack();
        telemetry.addData("Dist back ", distBack);

        while (distBack > back_distance) {
            // move back
            setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
            distBack = getDistanceBack();
        }

        setDriverMotorPower(0.0,0.0,0.0,0.0, 10);
    }

    public void setDistanceAndMoveForwardFromBackSensor(double wallDistance){
        double distBack = getDistanceBack();
        telemetry.addData("Dist Back ", distBack);
        telemetry.update();

        while (distBack < wallDistance) {
            // telemetry.addData("Dist front ", distFront);
            // move forward
            setDriverMotorPower(0.2,0.2,0.2,0.2);
            distBack = getDistanceBack();
        }

        setDriverMotorPower(0.0,0.0,0.0,0.0, 10);
    }

    public void setDuckDropperSpeed(double speed, int s){
        motorDuckDropper.setPower(-speed);
        sleep(s);

        motorDuckDropper.setPower(0);
        sleep(10);
    }

    public void STOP_AND_RESET() {
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmDuo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SET_TARGET_POWER_RUN(int position, double power) {
        motorArmDuo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorArmDuo.setTargetPosition(position);
        motorArm.setTargetPosition(-position);

        motorArmDuo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmDuo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorArm.setPower(-power);
        motorArmDuo.setPower(power);

        while (motorArmDuo.isBusy()) {
            telemetry.addData("left en", motorArm.getCurrentPosition());
            telemetry.addData("right en", motorArmDuo.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }

    public void SET_ARM_POWER_ZERO(){
        motorArmDuo.setPower(0);
        motorArm.setPower(0);
    }

    public void setClawPos(double pos1, double pos2) {
        servoClaw1.setPosition(pos1);
        servoClaw2.setPosition(pos2);
    }

    public void moveIntake(double intakeSpeed) {
        motorIntake.setPower(intakeSpeed);
    }

    public void releaseHoldGate() {
        servoHold.setPosition(0);
    }

    public void pushIntake() {
        servoPush.setPosition(1);
        sleep(100);
        servoPush.setPosition(0);
        sleep(100);
        servoPush.setPosition(1);
    }

    public void resetPushIntake() {
        servoPush.setPosition(0);
    }
}
