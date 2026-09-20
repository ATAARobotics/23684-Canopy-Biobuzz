package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Colour Sensor Test", group = "Tests")
public class ColourSensorTest extends SafeOpMode {
    private SafeDevice<NormalizedColorSensor> test_color;

    @Override
    protected void onSafeInit() {
        test_color = safeMap.device(NormalizedColorSensor.class, "colour");
    }

    @Override
    protected void onSafeLoop() {
        test_color.run(sensor -> {
            NormalizedRGBA colors = sensor.getNormalizedColors();

            // Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
        });
        telemetry.update();
    }


}
