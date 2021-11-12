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
import org.firstinspires.ftc.teamcode.pipelines.DuckDetector;

import java.util.Locale;

/*******************************************************************
 *     ___   __  ____________  _   ______  __  _______  __  _______ *
 *    /   | / / / /_  __/ __ \/ | / / __ \/  |/  / __ \/ / / / ___/ *
 *   / /| |/ / / / / / / / / /  |/ / / / / /|_/ / / / / / / /\__ \  *
 *  / ___ / /_/ / / / / /_/ / /|  / /_/ / /  / / /_/ / /_/ /___/ /  *
 * /_/  |_\____/ /_/  \____/_/ |_/\____/_/  /_/\____/\____//____/   *
 *******************************************************************/

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoBotv2")
public class AutoBotv2 extends AutoRobotStruct {
    InitCV AutoCVCMD;
    DuckDetector duckVision = new DuckDetector();;
    String position = "NOT FOUND";
    String direction = "LEFT";
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void detect(){
        if (position.equals("LEFT")) {
            // inverse from the box this correlates to because the phone is upside down
            telemetry.addData("Position: ", "RIGHT");
            telemetry.update();

            direction = "RIGHT";

        } else if (position.equals("RIGHT")) {
            // inverse from the box this correlates to because the phone is upside down
            telemetry.addData("Position: ", "LEFT");
            telemetry.update();

            direction = "LEFT";

        } else if (position.equals("MIDDLE")) {
            telemetry.addData("Position: ", "MIDDLE");
            telemetry.update();

            direction = "MIDDLE";

        } else {
            telemetry.addData("Position: ", "NOT FOUND");
            telemetry.update();
        }
    }

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        AutoCVCMD = new InitCV();
        AutoCVCMD.init(duckVision, cameraMonitorViewId);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

//        composeTelemetry();

        while(!isStarted()) {
            position = duckVision.getLoc();
            detect();
        }

        initRunner();

        waitForStart();
//        AutoCVCMD.stopStream();

//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (direction.equals("MIDDLE")) {
            while (opModeIsActive()) {
                telemetry.update();
                setClawPos(0.3);

                // move right
                translateRight(2);
                sleep(800);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(500);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // move arm down
                setArmSpeed(-0.5);
                sleep(2500);

                // release cube
                setClawPos(0.90);

                // force end of while loop
                requestOpModeStop();
            }
        }

        else if (direction.equals("LEFT")) {
            while (opModeIsActive()) {
                telemetry.update();
                setClawPos(0.3);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(100);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                translateRight(1.5);
                sleep(600);

                // move arm down
                setArmSpeed(0.5);
                sleep(2900);

                // release cube
                setClawPos(0.90);

                // force end of while loop
                requestOpModeStop();
            }
        }

        else {
            while (opModeIsActive()) {
                telemetry.update();
                setClawPos(0.3);

                // move right
                translateRight(2);
                sleep(800);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(500);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // move arm down
                setArmSpeed(-0.5);
                sleep(2500);

                // release cube
                setClawPos(0.90);

                // force end of while loop
                requestOpModeStop();
            }
        }
    }
//
//    void composeTelemetry() {
//        telemetry.addAction(new Runnable() {
//            @Override
//            public void run() {
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//            }
//        });
//
//        telemetry.addLine()
//                .addData("status", () -> imu.getSystemStatus().toShortString())
//                .addData("calib", () -> imu.getCalibrationStatus().toString());
//
//        telemetry.addLine()
//                .addData("heading/yaw", () -> formatAngle(angles.angleUnit, angles.firstAngle))
//                .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
//                .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));
//
//        telemetry.addLine()
//                .addData("grvty", () -> gravity.toString())
//                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f",
//                        Math.sqrt(gravity.xAccel * gravity.xAccel
//                                + gravity.yAccel * gravity.yAccel
//                                + gravity.zAccel * gravity.zAccel)));
//
//        telemetry.update();
//    }
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }
}
