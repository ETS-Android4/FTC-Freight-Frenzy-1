package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.RobotStruct;
// Game pad routing docs for reference: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

@TeleOp(name="DriverControl")
public class DriverControl extends RobotStruct {
    @Override
    public void loop() {
        double intakeSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
//        double rotate = gamepad1.left_stick_x;
        double armSpeed = gamepad2.left_stick_y;
        double duckDropperSpeed = gamepad2.right_trigger - gamepad2.left_trigger;

//        setDriverMotorPower(speed - rotate, speed + rotate, speed - rotate, speed + rotate);
        setDuckDropperSpeed(duckDropperSpeed);
        setIntakeSpeed(intakeSpeed);
        setArmSpeed(-armSpeed);
        initDriver();

        /*
        Servo claw positioned on the robot arm which opens and closes
         */

//        close
        if(gamepad2.a) {
            setClawPos(0.9, 0.01);
        }

//        open
        if(gamepad2.b) {
            setClawPos(0.75, 0.15);
        }
    }
}