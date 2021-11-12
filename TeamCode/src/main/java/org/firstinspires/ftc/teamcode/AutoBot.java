//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
//@Autonomous(name="AutoBot")
//public class AutoBot extends AutoRobotStruct {
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
//
//        }
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initRunner();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        AutoCVCMD = new InitCV();
//        AutoCVCMD.init(duckVision, cameraMonitorViewId);
//
//        while(!isStarted()){
//            position = duckVision.getLoc();
//            detect();
//        }
//
//        waitForStart();
//        AutoCVCMD.stopStream();
//
//        if (direction.equals("MIDDLE")) {
//            // move forward
//            setDriverMotorPower(0.5,0.5,0.5,0.5);
//            sleep(100);
//            setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//            // open claw
//            setClawPos(0.90);
//
//            // move arm down
//            setArmSpeed(0.5);
//            sleep(4000);
//
//            // grip cube
//            setClawPos(0.35);
//
//            // move arm up
//            setArmSpeed(-0.5);
//            sleep(2000);
//
//            // drive forward to the layered hub
//            setDriverMotorPower(0.5,0.5,0.5,0.5);
//            sleep(100);
//            setDriverMotorPower(0.0,0.0,0.0,0.0);
//
//            // move arm down to place cube
//            setArmSpeed(0.5);
//            sleep(2000);
//
//            // release cube
//            setClawPos(0.90);
//        }
//
//        else if (direction.equals("LEFT")) { }
//
//        else { }
//    }
//}
