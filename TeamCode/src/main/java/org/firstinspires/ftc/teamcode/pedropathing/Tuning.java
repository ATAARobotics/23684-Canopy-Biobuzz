package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedropathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedropathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedropathing.procedures.Tests;

public class Tuning {
    // Tuners go here

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
//    @Tuner
//    public static Procedure tests() {
//        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
//    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
}