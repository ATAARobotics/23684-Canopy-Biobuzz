package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * AprilTag cluster detection as a Synapse {@link Node}.
 *
 * <p>Polls an {@link AprilTagProcessor} at {@link #POLL_HZ}, converts every detection into a
 * {@link ClusterTarget} with its geometry already resolved, and publishes the results for other
 * nodes. Telemetry is written from this node, so it works from any OpMode that registers it.
 *
 * <h2>Turret yaw and the SDK's camera pose</h2>
 *
 * In SDK 12.0 {@code setCameraPose} exists <b>only</b> on {@code AprilTagProcessor.Builder} — the
 * processor itself cannot be re-aimed after it is built. So the camera pose is fixed here at the
 * mounting's pitch, roll and translation, and {@link VisionCameraPose#getYaw() the turret yaw} is
 * applied in {@link TargetingMath} instead, when resolving each detection. That means turret
 * rotation costs nothing at runtime: no portal rebuild, no dropped frames, no stream restart.
 *
 * <p>The practical consequence: {@link ClusterTarget#rawX rawX}/{@code rawY}/{@code rawZ} are the
 * SDK's numbers and ignore turret yaw, while {@link ClusterTarget#resolved} includes it. Always
 * consume {@code resolved} for aiming and driving.
 *
 * <p>Changing pitch, roll or translation has no runtime effect, because there is no supported way
 * to push them into a live processor.
 *
 * <h2>Published topics</h2>
 *
 * <ul>
 *   <li>{@code vision/clusterCount} — Integer, detections on the last poll</li>
 *   <li>{@code vision/usableCount} — Integer, how many are scorable and fresh</li>
 *   <li>{@code vision/hasTarget} — Boolean, whether a usable target exists</li>
 *   <li>{@code vision/best} — ClusterTarget, nearest usable cluster. Only published when one
 *       exists, since {@code publish} rejects null</li>
 *   <li>{@code vision/targets} — {@code List<ClusterTarget>}, every detection this poll</li>
 *   <li>{@code vision/decimation} — Integer, current decimation</li>
 *   <li>{@link #HEADING_TOPIC} — Double, robot heading in degrees CCW from field +X</li>
 * </ul>
 *
 * <h2>Subscribed topics</h2>
 *
 * <ul>
 *   <li>{@code vision/setDecimation} — Integer, trades detection range for frame rate</li>
 *   <li>{@code vision/nudgeCameraYaw} — Double, degrees to add to the turret yaw</li>
 *   <li>{@code vision/setCameraYaw} — Double, absolute turret yaw in degrees</li>
 *   <li>{@code vision/resetCameraPose} — Boolean, true restores the default extrinsics</li>
 * </ul>
 *
 * <p><b>Heading:</b> this robot has no odometry, so nothing supplies a true field heading yet.
 * {@link #DEFAULT_ROBOT_HEADING_DEG} is assumed, which makes {@code resolved.bearing} an aim error
 * relative to field +X rather than relative to wherever the robot is actually pointing. Once
 * odometry exists, publish heading to {@link #HEADING_TOPIC} and every number becomes correct with
 * no code change.
 */
public class AprilTagVision extends Node {

	/** Webcam name in the robot configuration. */
	public static final String DEFAULT_CAMERA_NAME = "gobilda_camera";

	/** Alliance used to decide whether a cluster is ours. */
	public static final char DEFAULT_ALLIANCE_LETTER = 'B';

	/** Detection poll rate. The camera streams faster than this; this is how often we read it. */
	public static final int POLL_HZ = 20;

	/**
	 * Decimation used for detection.
	 *
	 * <p>Decimation downsamples the image before searching for tags, so every step throws away
	 * resolution and blurs tag corners. That directly costs range and bearing accuracy, and it costs it
	 * unevenly: a small distant tag loses its corners entirely while a large close one survives. That
	 * shows up as readings that are sometimes nearly perfect and sometimes a few inches out.
	 *
	 * <p>1 processes the full 1280x800 frame, which is the most accurate setting. The cost is CPU on
	 * the RC, so the frame rate may drop; {@link #getActualFps()} and the staleness flag on each target
	 * will tell you if that has gone too far. Raise it with {@code vision/setDecimation} at runtime.
	 */
	public static final int DEFAULT_DECIMATION = 1;

	/**
	 * Tags per cluster in BIOBUZZ.
	 *
	 * <p>A constant because the SDK does not expose it: {@code AprilTagClusterMetadata.getMemberIds()}
	 * and its {@code clusterMembers} field are both non-public, so the count cannot be read from
	 * outside {@code org.firstinspires.ftc.vision.apriltag}. This is what lets telemetry say
	 * "2 of 4 tags" instead of a bare percentage, which is far easier to act on when a rail or the
	 * field wall is hiding half the cluster.
	 */
	public static final int TAGS_PER_CLUSTER = 4;

	/**
	 * MJPEG rather than the default YUY2. At this resolution YUY2 pushes far more data per frame,
	 * and MJPEG offloads the compression to the camera itself.
	 */
	public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG;

	/**
	 * Requested capture size.
	 *
	 * <p>Calibrated by us in {@code FtcRobotController/src/main/res/xml/teamwebcamcalibrations.xml} for USB
	 * {@code 0xC45:0x366}, which gives focal length {@code 908.758} and principal point
	 * {@code (696.345, 376.979)} at this size. Keep the two in sync: a size with no matching entry
	 * there has no intrinsics to scale from and falls back to a generic guess.
	 *
	 * <p>The file must live in FtcRobotController, not TeamCode. RobotCore's
	 * {@code CameraCalibrationHelper} reads calibrations through its own
	 * {@code com.qualcomm.robotcore.R.xml.teamwebcamcalibrations}, which resolves in the RobotCore
	 * package. A copy under TeamCode lands in a different resource package and is never read, so the
	 * processor silently falls back to {@code fx = 578.272} — about 57% under the real value, which
	 * makes every range come out roughly 0.57x too short.
	 */
	public static final int STREAM_WIDTH = 1280;
	public static final int STREAM_HEIGHT = 800;

	/** Topic for robot heading, in degrees CCW from field +X. */
	public static final String HEADING_TOPIC = "vision/robotHeading";

	/** Assumed heading until something publishes a real one. */
	public static final double DEFAULT_ROBOT_HEADING_DEG = 0.0;

	/**
	 * Minimum age at which a detection is considered stale.
	 *
	 * <p>This is a floor, not the real threshold. A detection is normally one frame interval old, so
	 * a fixed wall-clock limit would condemn perfectly good frames whenever the camera runs slowly.
	 * At {@link #DEFAULT_DECIMATION} of 1 over a 1280x800 stream the camera may only manage a few
	 * frames per second, leaving every detection hundreds of milliseconds old by definition.
	 *
	 * <p>See {@link #staleThresholdMs()} for the threshold actually applied.
	 */
	public static final double STALE_AFTER_MS = 250.0;

	/**
	 * How many frame intervals a detection may lag before it counts as stale.
	 *
	 * <p>Staleness should mean the processor has fallen behind, not that time has passed. One
	 * interval is normal; this many tolerates jitter and a dropped frame while still catching a
	 * genuinely stuck pipeline.
	 */
	public static final double STALE_FRAME_INTERVALS = 3.0;

	private final AprilTagProcessor aprilTag;
	private final VisionPortal visionPortal;
	private final VisionCameraPose cameraPose;
	private final PointOfInterest poi;
	private final char allianceLetter;

	/** Camera name in use, echoed into telemetry so the setup line is self-describing. */
	private final String cameraName;

	private volatile List<ClusterTarget> lastTargets = Collections.emptyList();
	private volatile ClusterTarget lastBest;
	private volatile int lastDecimation = DEFAULT_DECIMATION;
	private volatile double lastHeadingDeg = DEFAULT_ROBOT_HEADING_DEG;

	/** When true, telemetry includes the configuration block and raw SDK numbers. */
	private volatile boolean verbose;

	/** Focus mode the camera settled into, for telemetry. Null until attempted. */
	private volatile String focusMode = "unknown";

	/** Whether exposure and white balance were pinned to manual. */
	private volatile boolean exposureLocked;
	private volatile boolean whiteBalanceLocked;

	/**
	 * @param orchestrator bus to publish on
	 * @param hardwareMap used to resolve the webcam
	 * @param cameraPose camera extrinsics; only {@code yaw} has any runtime effect
	 * @param poi point of interest, disabled by default
	 * @param allianceLetter {@code 'R'} or {@code 'B'}, matched against cluster name prefixes
	 */
	public AprilTagVision(
			Orchestrator orchestrator,
			HardwareMap hardwareMap,
			VisionCameraPose cameraPose,
			PointOfInterest poi,
			char allianceLetter) {
		this(orchestrator, hardwareMap, cameraPose, poi, allianceLetter, DEFAULT_CAMERA_NAME);
	}

	/**
	 * @param cameraName webcam name in the robot configuration
	 */
	public AprilTagVision(
			Orchestrator orchestrator,
			HardwareMap hardwareMap,
			VisionCameraPose cameraPose,
			PointOfInterest poi,
			char allianceLetter,
			String cameraName) {
		super(orchestrator);

		this.cameraPose = cameraPose;
		this.poi = poi;
		this.allianceLetter = Character.toUpperCase(allianceLetter);
		this.cameraName = cameraName;

		// Build with the mounting's fixed pose. Turret yaw is deliberately left at its resting
		// value here and applied later in TargetingMath, because the processor cannot be re-aimed.
		this.aprilTag = new AprilTagProcessor.Builder()
				.setCameraPose(cameraPose.getPosition(), cameraPose.getAngles())
				.setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.setDrawAxes(false)
				.setDrawTagOutline(true)
				.setDrawCubeProjection(false)
				.setDrawTagID(true)
				.build();

		VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, cameraName))
				.setStreamFormat(STREAM_FORMAT)
				.setCameraResolution(new Size(STREAM_WIDTH, STREAM_HEIGHT))
				.enableLiveView(true)
				.setAutoStopLiveView(false);
		portalBuilder.addProcessor(aprilTag);
		this.visionPortal = portalBuilder.build();

		// Must be applied explicitly: the Builder has no decimation setter, and the processor's own
		// default is 3 regardless of stream resolution.
		aprilTag.setDecimation(DEFAULT_DECIMATION);

		lockCameraControls();

		orchestrator.publish(HEADING_TOPIC, DEFAULT_ROBOT_HEADING_DEG);
		orchestrator.publish("vision/decimation", lastDecimation);
		orchestrator.publish("vision/hasTarget", false);
		orchestrator.publish("vision/targets", lastTargets);
		orchestrator.publish("vision/clusterCount", 0);
		orchestrator.publish("vision/usableCount", 0);
	}

	/**
	 * Read detections, resolve geometry, publish results.
	 *
	 * <p>Runs on the scheduler pool rather than the hardware thread: the AprilTag processor does its
	 * own work asynchronously and this method only reads results, so it never blocks the bus.
	 */
	@RunPeriodically(hz = POLL_HZ)
	public void poll() {
		double heading = orchestrator.getLatestValue(HEADING_TOPIC, Double.class).orElse(DEFAULT_ROBOT_HEADING_DEG);
		this.lastHeadingDeg = heading;

		long now = System.nanoTime();
		List<AprilTagDetection> detections = aprilTag.getDetections();

		List<ClusterTarget> resolved = new ArrayList<>(detections.size());
		for (AprilTagDetection detection : detections) {
			ClusterTarget target = toClusterTarget(detection, heading, now);
			if (target != null) {
				resolved.add(target);
			}
		}

		// Nearest usable cluster wins, scored on resolved range so height is accounted for.
		ClusterTarget best = null;
		int usable = 0;
		for (ClusterTarget target : resolved) {
			if (target.usable()) {
				usable++;
				if (best == null || target.resolved.range < best.resolved.range) {
					best = target;
				}
			}
		}

		this.lastTargets = resolved;
		this.lastBest = best;

		orchestrator.publish("vision/targets", resolved);
		orchestrator.publish("vision/clusterCount", resolved.size());
		orchestrator.publish("vision/usableCount", usable);
		orchestrator.publish("vision/hasTarget", best != null);
		if (best != null) {
			// publish() rejects null, so only announce a target while we actually have one.
			orchestrator.publish("vision/best", best);
		}
	}

	/** Convert one SDK detection, or return null if we cannot make sense of it. */
	private ClusterTarget toClusterTarget(AprilTagDetection detection, double headingDeg, long nowNanos) {
		String name;
		String shortName;
		int percentFound;
		int tagsTotal = 0;

		if (detection instanceof AprilTagClusterDetection) {
			// A cluster detection is guaranteed to have metadata: the SDK could not have matched
			// those tags to a cluster without it, so unlike a single detection there is no null
			// check to do here.
			AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
			name = cluster.metadata.name;
			shortName = cluster.metadata.shortName;
			percentFound = cluster.percentClusterFound;
			tagsTotal = TAGS_PER_CLUSTER;
		} else if (detection instanceof AprilTagSingleDetection) {
			// BIOBUZZ defines clusters only, but handling singles keeps this node reusable on a
			// field that mixes both kinds.
			AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
			if (single.metadata == null) {
				return null;
			}
			name = single.metadata.name;
			shortName = name;
			percentFound = 100;
			tagsTotal = 1;
		} else {
			return null;
		}

		double rawX = detection.ftcPose.x;
		double rawY = detection.ftcPose.y;
		double rawZ = detection.ftcPose.z;
		double rawYaw = detection.ftcPose.yaw;
		double rawPitch = detection.ftcPose.pitch;
		double rawRoll = detection.ftcPose.roll;

		double absRoll = Math.abs(rawRoll);
		boolean upright = absRoll < 90.0;
		boolean allianceMatch = name.startsWith(String.valueOf(allianceLetter));

		// Turret yaw is folded in here, not in the SDK's pose.
		TargetingMath.ResolvedPoint resolvedPoint = TargetingMath.resolve(
				rawX,
				rawY,
				rawZ,
				rawYaw,
				cameraPose.getYaw(),
				cameraPose.getElevation(),
				headingDeg,
				poi);

		double ageMs = (nowNanos - detection.frameAcquisitionNanoTime) / 1_000_000.0;
		boolean stale = ageMs > staleThresholdMs();

		return new ClusterTarget(
				name,
				shortName,
				percentFound,
				tagsTotal,
				rawX,
				rawY,
				rawZ,
				rawYaw,
				rawPitch,
				rawRoll,
				detection.ftcPose.range,
				detection.ftcPose.bearing,
				detection.ftcPose.elevation,
				allianceMatch,
				upright,
				absRoll,
				resolvedPoint,
				detection.frameAcquisitionNanoTime,
				ageMs,
				stale);
	}

	/**
	 * Minimal telemetry: the target worth acting on, then one line for anything else. Everything
	 * constant during a match — camera name, stream format, mount angles — is deliberately hidden,
	 * because reading it every loop trains you to stop reading it.
	 *
	 * <p>Turn on {@link #setVerbose(boolean)} for bench work to get the configuration block and the
	 * SDK's raw numbers back.
	 */
	public void publishTelemetry(Telemetry telemetry) {
		ClusterTarget best = lastBest;

		// The measurement you act on goes first and stays at the top. Everything else is below it, so
		// toggling verbose can never push the numbers you are driving by off the screen.
		if (best == null) {
			telemetry.addLine("no usable cluster");
			for (ClusterTarget target : lastTargets) {
				telemetry.addLine("  " + target.compactLine());
			}
		} else {
			// Marker rather than a label, so it is obvious at a glance which mode is active without
			// costing a whole line.
			telemetry.addLine((verbose ? "** VERBOSE **  " : "") + best.headerLine());
			telemetry.addLine(best.actionLines());

			if (lastTargets.size() > 1) {
				List<ClusterTarget> others = new ArrayList<>(lastTargets);
				others.remove(best);
				others.sort(Comparator.comparingDouble(t -> t.resolved.range));
				for (ClusterTarget target : others) {
					telemetry.addLine("  " + target.compactLine());
				}
			}

			if (verbose) {
				telemetry.addLine(best.verboseBlock());
			}
		}

		if (verbose) {
			// Compact, and last, so it reads as an appendix rather than competing with the above.
			telemetry.addLine(String.format(
					Locale.US,
					"cfg %s %dx%d @%.0ffps decim %d  poi %s",
					STREAM_FORMAT,
					STREAM_WIDTH,
					STREAM_HEIGHT,
					getActualFps(),
					lastDecimation,
					poi.isEnabled() ? poi.toString() : "off"));
			telemetry.addLine(String.format(
					Locale.US,
					"cam tilt %+.1f yaw %+.1f roll %+.1f at %.1f,%.1f,%.1f  focus %s exp %s wb %s alliance %c",
					cameraPose.getElevation(),
					cameraPose.getYaw(),
					cameraPose.getRoll(),
					cameraPose.getX(),
					cameraPose.getY(),
					cameraPose.getZ(),
					focusMode,
					exposureLocked ? "manual" : "auto",
					whiteBalanceLocked ? "manual" : "auto",
					allianceLetter));
			telemetry.addLine(String.format(
					Locale.US,
					"head %+.1f%s   camera %s",
					lastHeadingDeg,
					lastHeadingDeg == DEFAULT_ROBOT_HEADING_DEG ? " ASSUMED" : "",
					cameraName));
		}
	}

	/** Show the configuration block and raw SDK numbers. Off by default. */
	public void setVerbose(boolean verbose) {
		this.verbose = verbose;
	}

	/**
	 * Stop the camera from retuning itself while the robot moves.
	 *
	 * <p>Autofocus is the important one. A focus change alters the lens-to-sensor distance, which
	 * changes the effective focal length, and range is directly proportional to focal length. So a
	 * camera refocusing as the target moves reports a scale that drifts frame to frame, which matches
	 * the symptom of readings that are sometimes nearly perfect and sometimes a few inches out.
	 * Pinning focus to fixed holds the scale steady.
	 *
	 * <p>Every control is optional hardware, so each is attempted independently and failures are
	 * recorded rather than thrown: a camera without autofocus should not stop the robot from running.
	 */
	private void lockCameraControls() {
		// ---- focus: the one that matters for range ----
		try {
			FocusControl focus = visionPortal.getCameraControl(FocusControl.class);
			if (focus != null) {
				for (FocusControl.Mode mode :
						new FocusControl.Mode[] {FocusControl.Mode.Fixed, FocusControl.Mode.Infinity}) {
					if (focus.isModeSupported(mode) && focus.setMode(mode)) {
						this.focusMode = mode.name();
						break;
					}
				}
			}
		} catch (Exception e) {
			this.focusMode = "unsupported";
			orchestrator.warn("Focus control unavailable: " + e.getMessage());
		}

		// ---- exposure and white balance: stop them hunting as the robot moves ----
		// Done inline rather than on the hardware thread: these are camera-control calls, not Lynx
		// bus traffic, and SafeOpMode's hardware thread only starts ticking once the OpMode loop
		// begins, which is after this constructor has already returned.
		try {
			ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
			if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
				exposure.setMode(ExposureControl.Mode.Manual);
				this.exposureLocked = true;
			}
		} catch (Exception e) {
			orchestrator.warn("Exposure control unavailable: " + e.getMessage());
		}

		try {
			WhiteBalanceControl balance = visionPortal.getCameraControl(WhiteBalanceControl.class);
			// Note the upper case: WhiteBalanceControl.Mode uses MANUAL where ExposureControl and
			// FocusControl use Manual. Inconsistent across the SDK. This control also lacks the
			// isModeSupported() the other two have, so setMode's return value is the only signal.
			if (balance != null && balance.setMode(WhiteBalanceControl.Mode.MANUAL)) {
				this.whiteBalanceLocked = true;
			}
		} catch (Exception e) {
			orchestrator.warn("White balance control unavailable: " + e.getMessage());
		}
	}

	/**
	 * Age past which a detection stops being trusted, in milliseconds.
	 *
	 * <p>Scaled to the camera's actual frame rate. At decimation 1 over a 1280x800 stream the frame
	 * interval can be several hundred milliseconds, and a fixed limit would then flag every detection
	 * as stale, which would make every target unusable and leave the driver station reporting "no
	 * usable cluster" forever while the vision was in fact working fine.
	 *
	 * <p>Falls back to {@link #STALE_AFTER_MS} when the frame rate is not known yet, which is the
	 * case briefly at startup.
	 */
	private double staleThresholdMs() {
		float fps = getActualFps();
		if (fps <= 0.0f) {
			return STALE_AFTER_MS;
		}
		double frameIntervalMs = 1000.0 / fps;
		return Math.max(STALE_AFTER_MS, STALE_FRAME_INTERVALS * frameIntervalMs);
	}

	/**
	 * Frame rate the camera actually achieved. SDK 12.0 exposes no way to request a frame rate, so
	 * this negotiated number is the only honest measure of whether the stream is healthy.
	 */
	public float getActualFps() {
		return visionPortal.getFps();
	}

	/** Latest detections, never null. */
	public List<ClusterTarget> getTargets() {
		return lastTargets;
	}

	/** Nearest usable cluster, or null if there is none. */
	public ClusterTarget getBest() {
		return lastBest;
	}

	public VisionCameraPose getCameraPose() {
		return cameraPose;
	}

	public PointOfInterest getPointOfInterest() {
		return poi;
	}

	/** Change decimation live: higher sees farther but updates slower. */
	@SubscribedTo(topic = "vision/setDecimation")
	public void onSetDecimation(Integer decimation) {
		if (decimation == null) {
			return;
		}
		int clamped = Math.max(1, Math.min(4, decimation));
		aprilTag.setDecimation(clamped);
		this.lastDecimation = clamped;
		orchestrator.publish("vision/decimation", clamped);
	}

	/** Add to the turret yaw without a redeploy. */
	@SubscribedTo(topic = "vision/nudgeCameraYaw")
	public void onNudgeCameraYaw(Double degrees) {
		if (degrees == null || degrees == 0.0) {
			return;
		}
		cameraPose.nudgeYaw(degrees);
	}

	/** Set the turret yaw absolutely, in degrees. */
	@SubscribedTo(topic = "vision/setCameraYaw")
	public void onSetCameraYaw(Double degrees) {
		if (degrees == null) {
			return;
		}
		cameraPose.setYaw(degrees);
	}

	/** Restore the default extrinsics. Yaw takes effect immediately; the rest needs a restart. */
	@SubscribedTo(topic = "vision/resetCameraPose")
	public void onResetCameraPose(Boolean trigger) {
		if (Boolean.TRUE.equals(trigger)) {
			cameraPose.resetToDefaults();
		}
	}

	/** Release the camera when the node is unregistered. */
	@Override
	public void close() {
		visionPortal.close();
	}
}