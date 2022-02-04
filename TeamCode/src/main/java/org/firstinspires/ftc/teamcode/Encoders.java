package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;

@Autonomous(name="Encoders")
public class Encoders extends AutoRobotStruct {
    @Override
    public void runOpMode() {
        initRunner();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

//        STOP_AND_RESET();

        waitForStart();

//        encoderDrive(DRIVE_SPEED,  50,  50, 1.5);
        SET_TARGET_POWER_RUN(-1400, -0.25);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
