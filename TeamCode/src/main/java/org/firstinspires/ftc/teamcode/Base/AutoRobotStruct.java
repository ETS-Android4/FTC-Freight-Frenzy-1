package org.firstinspires.ftc.teamcode.Base;

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
    private DcMotor motorDuckDropper;
    private DcMotorEx motorIntake;
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
        motorDuckDropper = hardwareMap.get(DcMotorEx.class, "motor duck dropper");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw1 = hardwareMap.get(Servo.class, "servo claw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo claw2");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distance back");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distance front");
        motorIntake = hardwareMap.get(DcMotorEx.class, "motor intake");


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
//        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorArmDuo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmDuo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SET_TARGET_POWER_RUN_DOWN(int position, double power) {
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

    public void HOLD_POWER_SET_ZERO(){
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
}
