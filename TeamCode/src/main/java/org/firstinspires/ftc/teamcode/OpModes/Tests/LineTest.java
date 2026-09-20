package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Line Test", group = "Tests")
public class LineTest extends SafeOpMode {
    private static final double DISTANCE = 48.0;

    private Follower follower;
    private Path forwardPath;
    private Path returnPath;
    private boolean forward = true;

    @Override
    protected void onSafeInit() {
        follower = Constants.create(safeMap);
        follower.setPose(Pose.zero());
        forwardPath = Paths.line(Pose.zero(), new Pose(DISTANCE, 0, 0)).constant(0);
        returnPath = Paths.line(new Pose(DISTANCE, 0, 0), Pose.zero()).constant(0);
    }

    @Override
    protected void onSafeStart() {
        follower.follow(forwardPath);
    }

    @Override
    protected void onSafeLoop() {
        follower.update();
        if (follower.atParametricEnd()) {
            if (forward) {
                follower.follow(returnPath);
            } else {
                follower.follow(forwardPath);
            }
            forward = !forward;
        }
    }
}