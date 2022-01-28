package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;

import java.util.Locale;

import static java.lang.Double.parseDouble;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "GyroDriving")
public class GyroDriving extends AutoRobotStruct {
    String heading;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override public void runOpMode() {
        initRunner();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Thread  driveThread = new DriveThread();
        driveThread.start();

        while (opModeIsActive()) {
            telemetry.update();
            heading = getAngle();

            if (parseDouble(heading) > 0.5){
                heading = getAngle();
                setDriverMotorPower(0.25,-0.25,0.25,-0.25);
                heading = getAngle();
            }

            if (parseDouble(heading) < 0.5) {
                heading = getAngle();
                setDriverMotorPower(-0.25,0.25,-0.25,0.25);
                heading = getAngle();
            }

            heading = getAngle();


//            driveThread.interrupt();
//            setDriverMotorPower(0,0,0,0);
//            sleep(100);
//
//            requestOpModeStop();
        }

//        turn program
//        while (opModeIsActive()) {
//            telemetry.update();
//            heading = getAngle();
//
//            while (parseDouble(heading) > -78) {
//                heading = getAngle();
//                telemetry.update();
////              turn right
//                setDriverMotorPower(0.25,-0.25,0.25,-0.25);
//                heading = getAngle();
//            }
//
//            setDriverMotorPower(0,0,0,0);
//            sleep(100);
//
//            requestOpModeStop();
//        }
    }

    void composeTelemetry() {
        telemetry.addAction(() -> {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        });

        telemetry.addLine()
                .addData("status", () -> imu.getSystemStatus().toShortString())
                .addData("calib", () -> imu.getCalibrationStatus().toString());

        telemetry.addLine()
                .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine()
                .addData("grvty", () -> gravity.toString())
                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity.xAccel*gravity.xAccel
                                + gravity.yAccel*gravity.yAccel
                                + gravity.zAccel*gravity.zAccel)));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public String getAngle() {
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }


    private class DriveThread extends Thread {
        public DriveThread() {
            System.out.println("Drive thread");
        }

        @Override
        public void run() {
            while (!isInterrupted()) {
                // translate
                setDriverMotorPower(0.25,-0.25,-0.25,0.25);
            }
        }
    }
}
