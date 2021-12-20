package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.pipelines.DuckDetector;

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


        while(!isStarted()) {
            position = duckVision.getLoc();
            detect();
        }

        initRunner();

        waitForStart();
//        AutoCVCMD.stopStream();

        if (direction.equals("MIDDLE")) {
            while (opModeIsActive()) {
                telemetry.update();
//                setClawPos(0.3);

//                setArmSpeed(0);
//                sleep(100);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(100);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(270);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // distance to move forward
                double distFront = getDistanceFront();
                telemetry.addData("Dist front ", distFront);

                while (distFront > 11.0) {
//                    telemetry.addData("Dist front ", distFront);
                    // move forward
                    setDriverMotorPower(0.25,0.25,0.25,0.25);
                    distFront = getDistanceFront();
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // distance to lower arm to
//                double dist = getDistanceArm();
//                while(dist > 11.0) {
//                    dist = getDistanceArm();
//
//                    // move arm down
//                    setArmSpeed(-0.3);
//                }
//                setArmSpeed(0);
//                sleep(200);

                // release cube
//                setClawPos(0.90);

//                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(700);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(310);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                //                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(500);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);


                // force end of while loop
                requestOpModeStop();
            }
        }

        else if (direction.equals("LEFT")) {
            while (opModeIsActive()) {
                telemetry.update();
//                setClawPos(0.3);

//                setArmSpeed(0);
//                sleep(100);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(100);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(270);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // distance to move forward
                double distFront = getDistanceFront();
                telemetry.addData("Dist front ", distFront);

                while (distFront > 15.0) {
//                    telemetry.addData("Dist front ", distFront);
                    // move forward
                    setDriverMotorPower(0.25,0.25,0.25,0.25);
                    distFront = getDistanceFront();
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // distance to lower arm to
//                double dist = getDistanceArm();
//                while(dist > 11.0) {
//                    dist = getDistanceArm();
//
//                    // move arm down
//                    setArmSpeed(-0.3);
//                }
//                setArmSpeed(0);
//                sleep(200);

                // release cube
//                setClawPos(0.90);

//                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(700);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(310);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                //                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(500);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);


                // force end of while loop
                requestOpModeStop();
            }
        }

        else {
            while (opModeIsActive()) {
                telemetry.update();
//                setClawPos(0.3);

//                setArmSpeed(0);
//                sleep(100);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(100);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(290);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // distance to move forward
                double distFront = getDistanceFront();
                telemetry.addData("Dist front ", distFront);

                while (distFront > 8.0) {
//                    telemetry.addData("Dist front ", distFront);
                    // move forward
                    setDriverMotorPower(0.25,0.25,0.25,0.25);
                    distFront = getDistanceFront();
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // distance to lower arm to
//                double dist = getDistanceArm();
//                while(dist > 11.0) {
//                    dist = getDistanceArm();
//
//                    // move arm down
//                    setArmSpeed(-0.3);
//                }
//                setArmSpeed(0);
//                sleep(200);

                // release cube
//                setClawPos(0.90);

//                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(700);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);

                // turn right
                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
                sleep(310);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                //                move back
                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
                sleep(500);
                setDriverMotorPower(0.0,0.0,0.0,0.0);
                sleep(100);


                // force end of while loop
                requestOpModeStop();
            }
        }
    }
}
