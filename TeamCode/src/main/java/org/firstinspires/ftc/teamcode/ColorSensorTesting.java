package org.firstinspires.ftc.teamcode;
import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled // stops this file from uploading to the rc phone
@TeleOp(name = "Color Sensor", group = "Sensor")
public class ColorSensorTesting extends LinearOpMode {
    private ColorSensor colorSensor;
    private DcMotorEx motorIntake;

    @Override
    public void runOpMode() {
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        boolean bPrevState = false;
        boolean bCurrState = false;

        boolean bLedOn = true;

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        colorSensor.enableLed(bLedOn);

        waitForStart();

        while (opModeIsActive()) {
            colorSensor.enableLed(bLedOn);

            bPrevState = bCurrState;
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

//            while (hsvValues[0] < 40 && (colorSensor.red() > 60 && colorSensor.red() < 100)) {
//                motorIntake(1.0);
//            }

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
    }

    public void moveIntake(double intakeSpeed) {
        motorIntake.setPower(intakeSpeed);
    }
}
