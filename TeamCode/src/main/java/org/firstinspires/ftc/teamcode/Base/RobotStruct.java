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

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorDuckDropper = hardwareMap.get(DcMotor.class, "motor duck dropper");
//        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
//        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw1 = hardwareMap.get(Servo.class, "servo claw1");
        servoClaw2 = hardwareMap.get(Servo.class, "servo claw2");

        motorIntake = hardwareMap.get(DcMotorEx.class, "motor intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {}

    public void initDriver(){
        float gamepad1LeftY = -gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;
        float gamepad2RightY = gamepad2.right_stick_y;
        float gamepad2LeftY = gamepad2.left_stick_y;
        float FrontRight = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackRight = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
    }

    public void setDriverMotorPower(double m) {
        motorFrontRight.setPower(m);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(m);
        motorBackRight.setPower(m);
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

    public void rightDiagUp(double m){
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(m);
    }

    public void leftDiagUp(double m){
        motorFrontRight.setPower(m);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(m);
        motorBackRight.setPower(0);
    }

    public void rightDiagBack(double m){
        motorFrontRight.setPower(-m);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(-m);
        motorBackRight.setPower(0);
    }

    public void leftDiagBack(double m){
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(-m);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(-m);
    }

    public void setDuckDropperSpeed(double speed){
        motorDuckDropper.setPower(-speed);
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
