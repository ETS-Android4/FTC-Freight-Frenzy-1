package org.firstinspires.ftc.teamcode.Base;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.InitCV;
//import org.firstinspires.ftc.teamcode.pipelines.PickupPosition;
//import org.openftc.easyopencv.OpenCvInternalCamera;

public class RobotStruct extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorDuckDropper;
    DcMotorEx motorArm;
    DcMotorEx motorArmDuo;
    Servo servoClaw;
//    OpenCvInternalCamera phoneCam;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor front right");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor front left");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor back left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor back right");
        motorDuckDropper = hardwareMap.get(DcMotor.class, "motor duck dropper");
        motorArm = hardwareMap.get(DcMotorEx.class, "motor arm");
        motorArmDuo = hardwareMap.get(DcMotorEx.class, "motor arm duo");
        servoClaw = hardwareMap.get(Servo.class, "servo claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

//        InitCV DC_CV = new InitCV();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        DC_CV.init(new PickupPosition(telemetry), cameraMonitorViewId);
    }

    @Override
    public void loop() {}

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackLeft.setPower(BLeftPower);
        motorBackRight.setPower(BRightPower);
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

    public void setDuckDropperSpeed(double speed){
        motorDuckDropper.setPower(-speed);
    }

    public void setArmSpeed(double speed) {
        motorArm.setPower(-speed);
        motorArmDuo.setPower(speed);
    }

    public void setClawPos(double pos) {
        servoClaw.setPosition(pos);
    }

//    public void sendAmpReading() {
//        telemetry.addData(
//                "Arm Current: ",
//                motorArm.getCurrent(CurrentUnit.AMPS)
//        );
//        telemetry.update();
//    }
}
