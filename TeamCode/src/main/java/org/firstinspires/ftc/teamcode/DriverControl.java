package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.RobotStruct;
// Game pad routing docs for reference: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

@TeleOp(name="DriverControl")
public class DriverControl extends RobotStruct {
    @Override
    public void loop() {
        double speed = gamepad1.right_trigger - gamepad1.left_trigger;
        double straight = gamepad1.left_stick_x;
        double armSpeed = gamepad1.left_stick_y;
        double duckDropperSpeed = 0.50 * (gamepad2.right_trigger - gamepad2.left_trigger);

        setDriverMotorPower(speed - straight, speed + straight, speed - straight, speed + straight);
        setDuckDropperSpeed(duckDropperSpeed);
        setArmSpeed(-armSpeed);

        // sendAmpReading();

        if(gamepad1.a) {
            setClawPos(0.30);
        }

        if(gamepad1.y) {
            setClawPos(0.50);
        }

        if(gamepad1.right_stick_x > 0 ) {
            translateRight(3);
        }

        if(gamepad1.right_stick_x < 0) {
            translateLeft(3);
        }
    }
}