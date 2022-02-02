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
    final double oneEightyDeg = 2 * -78; // used in a full rotation of the bot but takes into account speed and power
    double heading;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Thread driveThread = new DriveThread();
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
            // constant * (target-current position)
            while (opModeIsActive()) {
                telemetry.update();
                // get initial orientation
                heading = getAngle();
                // close claw to grab cube
                setClawPos(0.93, 0.07);
                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5, 135);
                // turn right
                turnRight(-20);
                // distance to move forward
                setDistanceAndMoveForward(10);
                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1200, -1.0);
                sleep(200);
                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);
                // move arm back up
                SET_TARGET_POWER_RUN_DOWN(700, -1.0);
                sleep(200);
                SET_ARM_POWER_ZERO();
                // adjust bot so heading is approx. zero
                double currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20,-0.20,0.20,-0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20,0.20,-0.20,0.20);
                        currentPosition = getAngle();
                    }
                }
                // back up
                setDistanceAndMoveBackward(6.5);
                sleep(10);
                // turn right
                turnRight(-78);
                // move back to hit duck dropper
                setDistanceAndMoveBackward(10);
                // spin motor
                setDuckDropperSpeed(0.75, 2000);
                sleep(10);
                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5, 500);
                // adjust bot so heading is approx. zero
                currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20,-0.20,0.20,-0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20,0.20,-0.20,0.20);
                        currentPosition = getAngle();
                    }
                }
                sleep(10);
                // rotation of the bot so  the intake faces the pit
                turnLeft(65);
                // back up to pit
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5, 2750);
                releaseHoldGate();
                // push down intake
                pushIntake();
                resetPushIntake();
                pushIntake();
                // translate and correct motion
                driveThread.start();
                setDriverMotorPower(-0.25,0.25,0.25,-0.25);
                sleep(2000);
                driveThread.interrupt();
//                correctionThread.interrupt();
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5, 2500);
                // force end of while loop
                requestOpModeStop();
            }
        }

        else if (direction.equals("MIDDLE")) {
            // constant * (target-current position)
            while (opModeIsActive()) {
                telemetry.update();
                // get initial orientation
                heading = getAngle();
                // close claw to grab cube
                setClawPos(0.93, 0.07);
                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5, 135);
                // turn right
                turnRight(-20);
                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1300, -1.0);
                sleep(200);
                // distance to move forward
                setDistanceAndMoveForward(10);
                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);
                setDriverMotorPower(-0.25, -0.25, -0.25, -0.25, 300);
                // move arm back up
                SET_TARGET_POWER_RUN_DOWN(700, -1.0);
                sleep(200);
                SET_ARM_POWER_ZERO();
                // adjust bot so heading is approx. zero
                double currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20,-0.20,0.20,-0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20,0.20,-0.20,0.20);
                        currentPosition = getAngle();
                    }
                }
                // back up
                setDistanceAndMoveBackward(6.5);
                sleep(10);
                // turn right
                turnRight(-78);
                // move back to hit duck dropper
                setDistanceAndMoveBackward(10);
                // spin motor
                setDuckDropperSpeed(0.75, 2000);
                sleep(10);
                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5, 500);
                // adjust bot so heading is approx. zero
                currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20,-0.20,0.20,-0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20,0.20,-0.20,0.20);
                        currentPosition = getAngle();
                    }
                }
                sleep(10);
                // rotation of the bot so  the intake faces the pit
                turnLeft(65);
                // back up to pit
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5, 2750);
                releaseHoldGate();
                // push down intake
                pushIntake();
                resetPushIntake();
                pushIntake();
                // translate and correct motion
                driveThread.start();
                setDriverMotorPower(-0.25,0.25,0.25,-0.25);
                sleep(2000);
                driveThread.interrupt();
//                correctionThread.interrupt();
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5, 2500);
                // force end of while loop
                requestOpModeStop();
            }
        }

        else {
            // constant * (target-current position)
            while (opModeIsActive()) {
                telemetry.update();
                // get initial orientation
                heading = getAngle();
                // close claw to grab cube
                setClawPos(0.93, 0.07);
                // move forward
                setDriverMotorPower(0.5, 0.5, 0.5, 0.5, 135);
                // turn right
                turnRight(-20);
                // lower arm
                SET_TARGET_POWER_RUN_DOWN(-1400, -1.0);
                sleep(200);
                // distance to move forward
                setDistanceAndMoveForward(10);
                // release cube
                setClawPos(0.87, 0.13);
                sleep(100);
                setDriverMotorPower(-0.25, -0.25, -0.25, -0.25, 300);
                // move arm back up
                SET_TARGET_POWER_RUN_DOWN(700, -1.0);
                sleep(200);
                SET_ARM_POWER_ZERO();
                // adjust bot so heading is approx. zero
                double currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20, -0.20, 0.20, -0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20, 0.20, -0.20, 0.20);
                        currentPosition = getAngle();
                    }
                }
                // back up
                setDistanceAndMoveBackward(6.5);
                sleep(10);
                // turn right
                turnRight(-78);
                // move back to hit duck dropper
                setDistanceAndMoveBackward(10);
                // spin motor
                setDuckDropperSpeed(0.75, 2000);
                sleep(10);
                // move forward
                setDriverMotorPower(0.5, 0.5, 0.5, 0.5, 500);
                // adjust bot so heading is approx. zero
                currentPosition = getAngle();
                while (currentPosition < -1.0 || currentPosition > 1.0) {
                    // self centering
                    telemetry.addData("heading", currentPosition);
                    telemetry.update();
                    if (currentPosition > 1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(0.20, -0.20, 0.20, -0.20);
                        currentPosition = getAngle();
                    }

                    if (currentPosition < -1.0) {
                        telemetry.addData("heading", currentPosition);
                        telemetry.update();
                        setDriverMotorPower(-0.20, 0.20, -0.20, 0.20);
                        currentPosition = getAngle();
                    }
                }
                sleep(10);
                // rotation of the bot so  the intake faces the pit
                turnLeft(65);
                // TODO: ADD TRANSLATE TOWARDS WALL ON NEXT LINE-> CHECK DIRECTION TO TURN
                // back up to pit
                setDriverMotorPower(-0.5, -0.5, -0.5, -0.5, 2750);
                releaseHoldGate();
                // push down intake
                pushIntake();
                resetPushIntake();
                pushIntake();
                // translate and correct motion
                driveThread.start();
                setDriverMotorPower(-0.25, 0.25, 0.25, -0.25);
                sleep(2000);
                driveThread.interrupt();
//                correctionThread.interrupt();
                setDriverMotorPower(-0.5, -0.5, -0.5, -0.5, 2500);
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

        while (currentPostition > intendedPosition) {
            telemetry.update();
            setDriverMotorPower(0.25,-0.25,0.25,-0.25);
            currentPostition = getAngle();
        }

        setDriverMotorPower(0, 0, 0, 0, 100);
    }

    public void turnLeft(double degreesToTurn) {
        double currentPosition = getAngle();
        double intendedPosition = currentPosition + degreesToTurn;

        while (currentPosition < intendedPosition) {
            telemetry.update();
            setDriverMotorPower(-0.25,0.25,-0.25,0.25);
            currentPosition = getAngle();
        }

        setDriverMotorPower(0, 0, 0, 0, 100);
    }

    private class DriveThread extends Thread {
        public DriveThread() {
            telemetry.addData("Starting DriveThread thread: ", "Success");
            telemetry.update();
        }

        @Override
        public void run() {
            double initialHeading = getAngle();
            double currentHeading = getAngle();

            try {
                while (!isInterrupted()) {
                    telemetry.update();
                    currentHeading = getAngle();

                    if (currentHeading > initialHeading){
//                        setDriverMotorPower(0.25,-0.25,0.25,-0.25);
                        setDriverMotorPower(-0.35,0.25,0.25,-0.35);
                        currentHeading = getAngle();
                    }

                    if (currentHeading < initialHeading) {
                        setDriverMotorPower(-0.25,0.35,0.35,-0.25);
                        currentHeading = getAngle();
                    }

                    telemetry.addData("current heading", currentHeading);
                    telemetry.update();

                    if(isInterrupted()) {
                        setDriverMotorPower(0,0,0,0);

                        throw new InterruptedException
                                ("Inner Exception: Shutting down driveThread and correctionThread!");
                    }
                }

                throw new InterruptedException("Outer Exception: Shutting down driveThread thread!");
            } catch (InterruptedException consumed) {}
        }
    }
}
