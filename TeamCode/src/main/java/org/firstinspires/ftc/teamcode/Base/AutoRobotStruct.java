package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoRobotStruct extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo servoClaw1;
    private Servo servoClaw2;
    DcMotor motorDuckDropper;

    private DcMotorEx motorArm;
    private DcMotorEx motorArmDuo;
    private DistanceSensor distanceBack;
    private DistanceSensor distanceFront;

    @Override
    public void runOpMode() throws InterruptedException { }

    public void initRunner() throws InternalError {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorDuckDropper = hardwareMap.get(DcMotor.class, "motor duck dropper");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw1 = hardwareMap.get(Servo.class, "servo claw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo claw2");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distance back");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distance front");


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

    public void setDuckDropperSpeed(double speed){
        motorDuckDropper.setPower(-speed);
    }

    public void STOP_AND_RESET() {
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmDuo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void SET_TARGET_POWER_RUN(int position, double power) {
        motorArm.setTargetPosition(position);
        motorArmDuo.setTargetPosition(position);

        motorArm.setPower(power);
        motorArmDuo.setPower(power);

        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmDuo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorArm.isBusy() || motorArmDuo.isBusy()){
            telemetry.addData("MOVING", "Arm");
            telemetry.update();
        }

        motorArm.setPower(0);
        motorArmDuo.setPower(0);
    }
//
//    public void setArmSpeed(double speed) {
//        motorArm.setPower(speed);
//        motorArmDuo.setPower(speed);
//    }

//    public void setClawPos(double pos) {
//        servoClaw.setPosition(pos);
//    }
}
