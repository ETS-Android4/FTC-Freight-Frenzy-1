//package org.firstinspires.ftc.teamcode;
//import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
//import org.firstinspires.ftc.teamcode.pipelines.DuckDetector;
//
///*******************************************************************
// *     ___   __  ____________  _   ______  __  _______  __  _______ *
// *    /   | / / / /_  __/ __ \/ | / / __ \/  |/  / __ \/ / / / ___/ *
// *   / /| |/ / / / / / / / / /  |/ / / / / /|_/ / / / / / / /\__ \  *
// *  / ___ / /_/ / / / / /_/ / /|  / /_/ / /  / / /_/ / /_/ /___/ /  *
// * /_/  |_\____/ /_/  \____/_/ |_/\____/_/  /_/\____/\____//____/   *
// *******************************************************************/
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DuckAutoRed")
//public class RedDuckAutoBotv2 extends AutoRobotStruct {
//    InitCV AutoCVCMD;
//    DuckDetector duckVision = new DuckDetector();
//    String position = "NOT FOUND";
//    String direction = "LEFT";
//
//    public void detect(){
//        if (position.equals("LEFT")) {
//            // inverse from the box this correlates to because the phone is upside down
//            telemetry.addData("Position: ", "RIGHT");
//            telemetry.update();
//
//            direction = "RIGHT";
//
//        } else if (position.equals("RIGHT")) {
//            // inverse from the box this correlates to because the phone is upside down
//            telemetry.addData("Position: ", "LEFT");
//            telemetry.update();
//
//            direction = "LEFT";
//
//        } else if (position.equals("MIDDLE")) {
//            telemetry.addData("Position: ", "MIDDLE");
//            telemetry.update();
//
//            direction = "MIDDLE";
//
//        } else {
//            telemetry.addData("Position: ", "NOT FOUND");
//            telemetry.update();
//        }
//    }
//
//    @Override public void runOpMode() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        AutoCVCMD = new InitCV();
//        AutoCVCMD.init(duckVision, cameraMonitorViewId);
//
//
//        while(!isStarted()) {
//            position = duckVision.getLoc();
//            detect();
//        }
//
//        initRunner();
//        STOP_AND_RESET();
//
//        waitForStart();
//
////        AutoCVCMD.stopStream();
//
//        if (direction.equals("RIGHT")) {
//            while (opModeIsActive()) {
//                telemetry.update();
//                setClawPos(0.93, 0.07);
//
//                // move forward
//                setDriverMotorPower(0.5,0.5,0.5,0.5);
//                sleep(135);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(220);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // distance to move forward
//                double distFront = getDistanceFront();
//                telemetry.addData("Dist front ", distFront);
//
//                while (distFront > 14.0) {
////                    telemetry.addData("Dist front ", distFront);
//                    // move forward
//                    setDriverMotorPower(0.25,0.25,0.25,0.25);
//                    distFront = getDistanceFront();
//                }
//
//                // stop motors
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                // lower arm
//                SET_TARGET_POWER_RUN_DOWN(-1100, -1.0);
//                sleep(200);
//
//                // release cube
//                setClawPos(0.87, 0.13);
//                sleep(100);
//
//                HOLD_POWER_SET_ZERO();
//
//                // distance to back up
//                double dist = getDistanceBack();
//                while(dist > 15.0) {
//                    dist = getDistanceBack();
//
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//                }
//                setDriverMotorPower(0,0,0,0);
//
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(400);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                //                move back
////                setDriverMotorPower(-0.5,-0.5,-0.5,-0.5);
////                sleep(500);
////                setDriverMotorPower(0.0,0.0,0.0,0.0);
////                sleep(100);
//
//                // distance to back up
//                dist = getDistanceBack();
//                while(dist > 12.0) {
//                    dist = getDistanceBack();
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//
//                    telemetry.addData("distance", dist);
//
//                    telemetry.update();
//                }
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(200);
//
//                setDuckDropperSpeed(0.10);
//                sleep(2000);
//                setDuckDropperSpeed(0.0);
//                sleep(100);
//
//                // move forward
//                setDriverMotorPower(1.0,1.0,1.0,1.0);
//                sleep(3000);
//
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                setDriverMotorPower(-0.5,0.5,-0.5,0.5);
//
//                // force end of while loop
//                requestOpModeStop();
//            }
//        }
//
//        else if (direction.equals("MIDDLE")) {
//            while (opModeIsActive()) {
//                telemetry.update();
//                setClawPos(0.93, 0.07);
//
//                // move forward
//                setDriverMotorPower(0.5,0.5,0.5,0.5);
//                sleep(135);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(220);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // distance to move forward
//                double distFront = getDistanceFront();
//                telemetry.addData("Dist front ", distFront);
//
//                while (distFront > 18.0) {
////                    telemetry.addData("Dist front ", distFront);
//                    // move forward
//                    setDriverMotorPower(0.25,0.25,0.25,0.25);
//                    distFront = getDistanceFront();
//                }
//
//                // stop motors
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                // lower arm
//                SET_TARGET_POWER_RUN_DOWN(-1440, -0.3);
//                sleep(200);
//
//                setDriverMotorPower(0.25,0.25,0.25,0.25);
//                sleep(250);
//
//                // release cube
//                setClawPos(0.87, 0.13);
//                sleep(100);
//
//                HOLD_POWER_SET_ZERO();
//
//                // distance to back up
//                double dist = getDistanceBack();
//                while(dist > 15.0) {
//                    dist = getDistanceBack();
//
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//                }
//                setDriverMotorPower(0,0,0,0);
//
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(400);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//
//                // distance to back up
//                dist = getDistanceBack();
//                while(dist > 12.0) {
//                    dist = getDistanceBack();
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//
//                    telemetry.addData("distance", dist);
//
//                    telemetry.update();
//                }
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(200);
//
//                setDuckDropperSpeed(0.25);
//                sleep(2000);
//                setDuckDropperSpeed(0.0);
//                sleep(100);
//
//                // move forward
//                setDriverMotorPower(1.0,1.0,1.0,1.0);
//                sleep(3000);
//
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                setDriverMotorPower(-0.5,0.5,-0.5,0.5);
//
//                // force end of while loop
//                requestOpModeStop();
//            }
//        }
//
//        else {
//            while (opModeIsActive()) {
//                telemetry.update();
//                setClawPos(0.93, 0.07);
//
//                // move forward
//                setDriverMotorPower(0.5,0.5,0.5,0.5);
//                sleep(135);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(220);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//                // distance to move forward
//                double distFront = getDistanceFront();
//                telemetry.addData("Dist front ", distFront);
//
//                while (distFront > 14.0) {
////                    telemetry.addData("Dist front ", distFront);
//                    // move forward
//                    setDriverMotorPower(0.25,0.25,0.25,0.25);
//                    distFront = getDistanceFront();
//                }
//
//                // stop motors
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                // lower arm
//                SET_TARGET_POWER_RUN_DOWN(-1600, -0.3);
//                sleep(200);
//
//                setDriverMotorPower(0.25,0.25,0.25,0.25);
//                sleep(250);
//
//                // release cube
//                setClawPos(0.87, 0.13);
//                sleep(100);
//
//                HOLD_POWER_SET_ZERO();
//
//                // distance to back up
//                double dist = getDistanceBack();
//                while(dist > 15.0) {
//                    dist = getDistanceBack();
//
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//                }
//                setDriverMotorPower(0,0,0,0);
//
//
//                // turn right
//                setDriverMotorPower(0.5,-0.5,0.5,-0.5);
//                sleep(400);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//
//                // distance to back up
//                dist = getDistanceBack();
//                while(dist > 12.0) {
//                    dist = getDistanceBack();
//                    // move robot back
//                    setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);
//
//                    telemetry.addData("distance", dist);
//
//                    telemetry.update();
//                }
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(200);
//
//                setDuckDropperSpeed(0.25);
//                sleep(2000);
//                setDuckDropperSpeed(0.0);
//                sleep(100);
//
//                // move forward
//                setDriverMotorPower(1.0,1.0,1.0,1.0);
//                sleep(3000);
//
//                setDriverMotorPower(0.0,0.0,0.0,0.0);
//                sleep(100);
//
//                setDriverMotorPower(-0.5,0.5,-0.5,0.5);
//
//                // force end of while loop
//                requestOpModeStop();
//            }
//        }
//    }
//}
