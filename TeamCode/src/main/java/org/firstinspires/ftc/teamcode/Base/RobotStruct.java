package org.firstinspires.ftc.teamcode.Base;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotStruct extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorDuckDropper;
    DcMotorEx motorArm;
    DcMotorEx motorArmDuo;
    DcMotor motorIntake;
    Servo servoClaw1;
    Servo servoClaw2;
    Servo servoHold;
    Servo servoPush;


    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorDuckDropper = hardwareMap.get(DcMotor.class, "motor duck dropper");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        motorIntake = hardwareMap.get(DcMotorEx.class, "motor intake");
        servoClaw1 = hardwareMap.get(Servo.class, "servo claw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo claw2");
        servoHold = hardwareMap.get(Servo.class, "servo Hold");
        servoHold = hardwareMap.get(Servo.class, "servo Push");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {}

    public void initDriver(){
//        removed a neg from line 50 might need to add back
        float gamepad1LeftY = gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = -gamepad1.right_stick_x;
        float frontRightPower = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
        float frontLeftPower = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float backLeftPower = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float backRightPower = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
    }

    public void setDriverPowerZERO() {
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
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

//    public void rightDiagUp(double m){
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(m);
//        motorBackLeft.setPower(0);
//        motorBackRight.setPower(m);
//    }
//
//    public void leftDiagUp(double m){
//        motorFrontRight.setPower(m);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(m);
//        motorBackRight.setPower(0);
//    }
//
//    public void rightDiagBack(double m){
//        motorFrontRight.setPower(-m);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(-m);
//        motorBackRight.setPower(0);
//    }
//
//    public void leftDiagBack(double m){
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(-m);
//        motorBackLeft.setPower(0);
//        motorBackRight.setPower(-m);
//    }

    public void setDuckDropperSpeed(double speed){
        motorDuckDropper.setPower(- (0.25) * speed);
    }

    public void setIntakeSpeed(double speed){
        motorIntake.setPower(-speed);
    }

    public void setArmSpeed(double speed) {
        motorArm.setPower(-speed);
        motorArmDuo.setPower(speed);
    }

    public void setClawPos(double pos1, double pos2) {
        servoClaw1.setPosition(pos1);
        servoClaw2.setPosition(pos2);
    }

    public void sendAmpReading() {
        telemetry.addData(
                "Arm Current: ",
                motorArm.getCurrent(CurrentUnit.AMPS)
        );
        telemetry.update();
    }
}
