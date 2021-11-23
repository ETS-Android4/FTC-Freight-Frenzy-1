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
    private DcMotorEx motorArm;
    private DcMotorEx motorArmDuo;
    private Servo servoClaw;
    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange1;

    @Override
    public void runOpMode() throws InterruptedException { }

    public void initRunner() throws InternalError {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw = hardwareMap.get(Servo.class, "servo claw");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensor_range1");


        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getDistanceFront() {
        return sensorRange1.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceArm() {
        return sensorRange.getDistance(DistanceUnit.INCH);
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
    }

    public void setArmSpeed(double speed) {
        motorArm.setPower(speed);
        motorArmDuo.setPower(speed);
    }

    public void setClawPos(double pos) {
        servoClaw.setPosition(pos);
    }

    public void translateRight(double m) {
        motorFrontRight.setPower(-m);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(-m);
        motorBackRight.setPower(m);
    }

    public void translateLeft(double m) {
        motorFrontRight.setPower(m);
        motorFrontLeft.setPower(-m);
        motorBackLeft.setPower(m);
        motorBackRight.setPower(-m);
    }
}
