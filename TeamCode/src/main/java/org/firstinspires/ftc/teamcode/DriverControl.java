package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.RobotStruct;
// Game pad routing docs for reference: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

@TeleOp(name="DriverControl")
public class DriverControl extends RobotStruct {
    @Override
    public void loop() {

//        double rotate = gamepad1.left_stick_x;
        double armSpeed = 0.75 * gamepad2.left_stick_y;
        double duckDropperSpeed = gamepad2.right_trigger - gamepad2.left_trigger;

//        setDriverMotorPower(speed - rotate, speed + rotate, speed - rotate, speed + rotate);
        setDuckDropperSpeed(duckDropperSpeed);
        setArmSpeed(-armSpeed);
        initDriver();


        while (gamepad1.dpad_down){
            setDriverMotorPower(0.25,0.25,0.25, 0.25);

            if (!gamepad1.dpad_down){
                setDriverPowerZERO();
            }
        }

        while (gamepad1.dpad_up){
            setDriverMotorPower(-0.25,-0.25,-0.25,-0.25);

            if (!gamepad1.dpad_up){
                setDriverPowerZERO();
            }
        }

        while (gamepad1.dpad_right){
            translateRight(0.25);

            if (!gamepad1.dpad_right){
                translateRight(0);
            }
        }

        while (gamepad1.dpad_left){
            translateLeft(0.25);

            if (!gamepad1.dpad_left){
                translateLeft(0);
            }
        }

        if (!gamepad2.dpad_up){
            setIntakeSpeed(0);
        }

        if (!gamepad2.dpad_down){
            setIntakeSpeed(0);
        }

        while (gamepad2.dpad_up){
            setIntakeSpeed(0.5);

            if (!gamepad2.dpad_up){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_down){
            setIntakeSpeed(-0.5);

            if (!gamepad2.dpad_down){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_up && gamepad2.y){
            setIntakeSpeed(0.5);

            if (!gamepad2.dpad_up){
                setIntakeSpeed(0);
            }
        }

        while (gamepad2.dpad_down && gamepad2.y){
            setIntakeSpeed(-0.5);

            if (!gamepad2.dpad_down) {
                setIntakeSpeed(0);
            }
        }

//        close
        if(gamepad2.a) {
            setClawPos(0.91, 0.09);
        }

//        open
        if(gamepad2.b) {
            setClawPos(0.85, 0.15);
        }

        if (gamepad2.x) {
            setHoldPosition(0);

        }

        if (gamepad2.y) {
            setPushPosition(0.1);
        }
    }
}