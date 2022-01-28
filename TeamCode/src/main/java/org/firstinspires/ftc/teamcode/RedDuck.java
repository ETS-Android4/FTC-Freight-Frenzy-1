package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.pipelines.DuckDetector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
import static java.lang.Double.parseDouble;

/*******************************************************************
 *     ___   __  ____________  _   ______  __  _______  __  _______ *
 *    /   | / / / /_  __/ __ \/ | / / __ \/  |/  / __ \/ / / / ___/ *
 *   / /| |/ / / / / / / / / /  |/ / / / / /|_/ / / / / / / /\__ \  *
 *  / ___ / /_/ / / / / /_/ / /|  / /_/ / /  / / /_/ / /_/ /___/ /  *
 * /_/  |_\____/ /_/  \____/_/ |_/\____/_/  /_/\____/\____//____/   *
 *******************************************************************/

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Duck Auto")
public class RedDuck extends AutoRobotStruct {
    InitCV AutoCVCMD;
    DuckDetector duckVision = new DuckDetector();
    String position = "NOT FOUND";
    String direction = "LEFT";
    final double oneEightyDeg = 2 * -78;
    double heading;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    Thread driveThread = new DriveThread();
//    Thread correctionThread = new CorrectionThread();

    public void detect(){
        if (position.equals("LEFT")) {
            // inverse position from the box this correlates to because the phone is upside down
            telemetry.addData("Position: ", "RIGHT");
            telemetry.update();

            direction = "RIGHT";

        } else if (position.equals("RIGHT")) {
            // inverse position from the box this correlates to because the phone is upside down
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

        while(!isStarted()) {
            position = duckVision.getLoc();
            detect();
        }

        // init robot hardware map and internal expansion hub gyro
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

        STOP_AND_RESET();

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // AutoCVCMD.stopStream();

        if (direction.equals("RIGHT")) {
            while (opModeIsActive()) {
                telemetry.update();
                // get initial orientation
                heading = getAngle();
                // close claw to grab cube
                setClawPos(0.93, 0.07);
                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5, 135);
                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5, 220);
                // distance to move forward
                setDistanceAndMoveForward(14.0);
                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1100, -1.0);
                sleep(200);
                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);
                SET_ARM_POWER_ZERO();
                // center self to approx. 0
                double currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 0.5){
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20,-0.20,0.20,-0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -0.5) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20,0.20,-0.20,0.20);
                        currentPosition = getAngle();
                    }
                }
                // adjust bot so heading is approx. zero
//                positionHeadingAtZero(); // TODO SANITY CHECK: BOT SHOULD ROTATE LEFT
                // back up
                setDistanceAndMoveBackward(15.0);
                // turn right
                turnRight(-78); // -78 deg is a 90 deg turn when we factor in the motor power
                // move back to hit duck dropper
                setDistanceAndMoveBackward(12);
                // spin motor
                // setDuckDropperSpeed(-0.20, 2000);
                // move forward
                setDriverMotorPower(1.0,1.0,1.0,1.0, 500);
                // complete rotation of the bot so  the intake faces the pit
                turnRight(oneEightyDeg);
                // TODO: ADD TRANSLATE TOWARDS WALL ON NEXT LINE-> CHECK DIRECTION TO TURN
                // back up to pit
                setDriverMotorPower(-1.0,-1.0,1.0,-1.0, 2500);
                releaseHoldGate();
                pushIntake();
                // force end of while loop
                requestOpModeStop();
            }
        }

        else if (direction.equals("MIDDLE")) {
            while (opModeIsActive()) {
                telemetry.update();
                setClawPos(0.93, 0.07);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(135);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(220);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // distance to move forward
                double distFront = getDistanceFront();
                telemetry.addData("Dist front ", distFront);

                while (distFront > 18.0) {
//                    telemetry.addData("Dist front ", distFront);
                    // move forward
                    setDriverMotorPower(0.25,0.25,0.25,0.25);
                    distFront = getDistanceFront();
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1440, -0.3);
                sleep(200);

                setDriverMotorPower(0.25,0.25,0.25,0.25);
                sleep(250);

                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);

                SET_ARM_POWER_ZERO();

                // distance to back up
                double dist = getDistanceBack();
                while(dist > 15.0) {
                    dist = getDistanceBack();

                    // move robot back
                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
                }
                setDriverMotorPower(0,0,0,0);


                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(400);
                setDriverMotorPower(0.0,0.0,0.0,0.0);


                // distance to back up
                dist = getDistanceBack();
                while(dist > 12.0) {
                    dist = getDistanceBack();
                    // move robot back
                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);

                    telemetry.addData("distance", dist);

                    telemetry.update();
                }
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(200);

                setDuckDropperSpeed(0.25, 2000);

                // move forward
                setDriverMotorPower(1.0,1.0,1.0,1.0);
                sleep(3000);

                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                setDriverMotorPower(-0.5,0.5,-0.5,0.5);

                // force end of while loop
                requestOpModeStop();
            }
        }

        else {
            while (opModeIsActive()) {
                telemetry.update();
                setClawPos(0.93, 0.07);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(135);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(220);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // distance to move forward
                double distFront = getDistanceFront();
                telemetry.addData("Dist front ", distFront);

                while (distFront > 14.0) {
//                    telemetry.addData("Dist front ", distFront);
                    // move forward
                    setDriverMotorPower(0.25,0.25,0.25,0.25);
                    distFront = getDistanceFront();
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1600, -0.3);
                sleep(200);

                setDriverMotorPower(0.25,0.25,0.25,0.25);
                sleep(250);

                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);

                SET_ARM_POWER_ZERO();

                // distance to back up
                double dist = getDistanceBack();
                while(dist > 15.0) {
                    dist = getDistanceBack();

                    // move robot back
                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
                }
                setDriverMotorPower(0,0,0,0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(400);
                setDriverMotorPower(0.0,0.0,0.0,0.0);


                // distance to back up
                dist = getDistanceBack();
                while(dist > 12.0) {
                    dist = getDistanceBack();
                    // move robot back
                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);

                    telemetry.addData("distance", dist);

                    telemetry.update();
                }
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(200);

                setDuckDropperSpeed(0.25, 2000);

                // move forward
                setDriverMotorPower(1.0,1.0,1.0,1.0);
                sleep(3000);

                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                setDriverMotorPower(-0.5,0.5,-0.5,0.5);

                // force end of while loop
                requestOpModeStop();
            }
        }
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

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getAngle() {
        return parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

//    public void positionHeadingAtZero() {
//        // self centering
//        telemetry.addData("heading", currentPosition);
//        if (currentPosition > 0.5){
//            telemetry.addData("heading", currentPosition);
//            setDriverMotorPower(0.20,-0.20,0.20,-0.20);
//            currentPosition = getAngle();
//        }
//
//        if (currentPosition < -0.5) {
//            telemetry.addData("heading", currentPosition);
//            setDriverMotorPower(-0.20,0.20,-0.20,0.20);
//            currentPosition = getAngle();
//        }
//    }

    public void turnRight(double degreesToTurn) {
        double currentPostition = getAngle();
        double intendedPosition = currentPostition + degreesToTurn;

        while (currentPostition < intendedPosition) {
            telemetry.update();
            setDriverMotorPower(0.25,-0.25,0.25,-0.25);
            currentPostition = getAngle();
        }

        setDriverMotorPower(0, 0, 0, 0, 100);
    }

//    private class DriveThread extends Thread {
//        public DriveThread() {
//            telemetry.addData("Starting DriveThread thread: ", "Success");
//            telemetry.update();
//        }
//
//        @Override
//        public void run() {
//
//            try {
//                while (!isInterrupted()) {
//                    // translate
//                    setDriverMotorPower(0.25,-0.25,-0.25,0.25);
//
//                    if(isInterrupted()){
//                        correctionThread.interrupt();
//                        setDriverMotorPower(0,0,0,0);
//
//                        throw new InterruptedException
//                                ("Inner Exception: Shutting down driveThread and correctionThread!");
//                    }
//                }
//
//                throw new InterruptedException("Outer Exception: Shutting down driveThread thread!");
//            } catch (InterruptedException consumed) {
//                correctionThread.interrupt();
//            }
//        }
//    }
//
//    private class CorrectionThread extends Thread {
//        public CorrectionThread() {
//            telemetry.addData("Starting DiverCorrection thread: ", "Success");
//            telemetry.update();
//        }
//
//        @Override
//        public void run() {
//            double initialPosition = getAngle();
//            double newPosition = getAngle();
//
//            while (!isInterrupted()) {
//                // translate
//                // adjust any inaccuracies as we translate
//                if (newPosition > initialPosition){
//                    setDriverMotorPower(0.25,-0.25,0.25,-0.25);
//                    newPosition = getAngle();
//                }
//
//                if (newPosition < initialPosition) {
//                    setDriverMotorPower(-0.25,0.25,-0.25,0.25);
//                    newPosition = getAngle();
//                }
//            }
//        }
//    }

//    private void translateLeft(int s) {
//        // start translating
//        driveThread.start();
//        sleep(s);
//        try {
//            driveThread.interrupt();
//            telemetry.addData("driveThread state:", "TERMINATED");
//        } catch (Exception e) {
//            telemetry.addData("driveThread state:", "TERMINATED");
//        }
//
//        try {
//            correctionThread.interrupt();
//            telemetry.addData("correctionThread state:", "TERMINATED");
//        } catch (Exception e) {
//            telemetry.addData("correctionThread state:", "TERMINATED");
//        }
//    }
}
