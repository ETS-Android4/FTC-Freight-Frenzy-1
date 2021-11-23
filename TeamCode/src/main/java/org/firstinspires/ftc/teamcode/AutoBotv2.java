package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
                setClawPos(0.3);

                // move forward
                setDriverMotorPower(0.5,0.5,0.5,0.5);
                sleep(100);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // turn right
                setDriverMotorPower(-0.5,0.5,-0.5,0.5);
                sleep(300);
                setDriverMotorPower(0.0,0.0,0.0,0.0);

//                // move forward
//                setDriverMotorPower(0.5,0.5,0.5,0.5);
//                sleep(600);
//                setDriverMotorPower(0.0,0.0,0.0,0.0);

                // move arm down
                setArmSpeed(-0.5);
                sleep(1200);

                while (getDistanceFront() > 11) {
                    // move forward
                    setDriverMotorPower(0.5,0.5,0.5,0.5);
                }

                // stop motors
                setDriverMotorPower(0.0,0.0,0.0,0.0);

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
}
