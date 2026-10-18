package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.util.Size;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
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

/**
 * AprilTag cluster detection as a Synapse Node that publishes ClusterTarget; consume {@code resolved}
 * for aiming, since turret yaw is folded in there and not in the raw SDK pose.
 */
public class AprilTagVision extends Node {

	public static final String DEFAULT_CAMERA_NAME = "gobilda_camera";

	public static final char DEFAULT_ALLIANCE_LETTER = 'B';

	public static final int POLL_HZ = 20;

	/** 1 = full 1280x800 frame, most accurate; higher decimation loses small distant tags first, so
	 * range and bearing accuracy degrade unevenly while the frame rate improves. */
	public static final int DEFAULT_DECIMATION = 1;

	/** BIOBUZZ tags per cluster; the SDK keeps cluster member lists non-public, so this is hard-coded. */
	public static final int TAGS_PER_CLUSTER = 4;

	/** MJPEG rather than the default YUY2: far less data per frame at this resolution. */
	public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG;

	/** 1280x800, calibrated in FtcRobotController's teamwebcamcalibrations.xml (RobotCore reads it
	 * from its own resource package); without that entry fx falls back to 578.272, ranges ~0.57x short. */
	public static final int STREAM_WIDTH = 1280;
	public static final int STREAM_HEIGHT = 800;

	/** Robot heading, in degrees CCW from field +X. */
	public static final String HEADING_TOPIC = "vision/robotHeading";

	/** No odometry exists, so while this is assumed, resolved.bearing is an aim error from field +X
	 * rather than from where the robot is actually pointing. */
	public static final double DEFAULT_ROBOT_HEADING_DEG = 0.0;

	/** Floor on the staleness limit, not the real threshold: a fixed wall-clock limit would condemn
	 * perfectly good frames whenever the camera runs slowly. See staleThresholdMs(). */
	public static final double STALE_AFTER_MS = 250.0;

	/** Frame intervals a detection may lag before it counts as stale; one interval is normal. */
	public static final double STALE_FRAME_INTERVALS = 3.0;

	private final AprilTagProcessor aprilTag;
	private final VisionPortal visionPortal;
	private final VisionCameraPose cameraPose;
	private final PointOfInterest poi;
	private final char allianceLetter;

	private final String cameraName;

	private volatile List<ClusterTarget> lastTargets = Collections.emptyList();
	private volatile ClusterTarget lastBest;
	private volatile int lastDecimation = DEFAULT_DECIMATION;
	private volatile double lastHeadingDeg = DEFAULT_ROBOT_HEADING_DEG;

	private volatile boolean verbose;

	private volatile String focusMode = "unknown";

	private volatile boolean exposureLocked;
	private volatile boolean whiteBalanceLocked;

	/** @param cameraPose extrinsics; only yaw has any runtime effect */
	public AprilTagVision(
			Orchestrator orchestrator,
			HardwareMap hardwareMap,
			VisionCameraPose cameraPose,
			PointOfInterest poi,
			char allianceLetter) {
		this(orchestrator, hardwareMap, cameraPose, poi, allianceLetter, DEFAULT_CAMERA_NAME);
	}

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

		// Fixed mounting pose: the processor cannot be re-aimed after build, so turret yaw is
		// deliberately left at rest here and applied in TargetingMath instead.
		this.aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPose.getPosition(), cameraPose.getAngles())
				.setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.setDrawAxes(false)
				.setDrawTagOutline(true)
				.setDrawCubeProjection(false)
				.setDrawTagID(true)
				.build();

		VisionPortal.Builder portalBuilder =
				new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, cameraName))
						.setStreamFormat(STREAM_FORMAT)
						.setCameraResolution(new Size(STREAM_WIDTH, STREAM_HEIGHT))
						.enableLiveView(true)
						.setAutoStopLiveView(false);
		portalBuilder.addProcessor(aprilTag);
		this.visionPortal = portalBuilder.build();

		// The Builder has no decimation setter, and the processor's own default is 3.
		aprilTag.setDecimation(DEFAULT_DECIMATION);

		lockCameraControls();

		orchestrator.publish(HEADING_TOPIC, DEFAULT_ROBOT_HEADING_DEG);
		orchestrator.publish("vision/decimation", lastDecimation);
		orchestrator.publish("vision/hasTarget", false);
		orchestrator.publish("vision/targets", lastTargets);
		orchestrator.publish("vision/clusterCount", 0);
		orchestrator.publish("vision/usableCount", 0);
	}

	/** Reads detections, resolves geometry and publishes them. Runs on the scheduler pool, not the
	 * hardware thread, so it never blocks the bus. */
	@RunPeriodically(hz = POLL_HZ)
	public void poll() {
		double heading =
				orchestrator.getLatestValue(HEADING_TOPIC, Double.class).orElse(DEFAULT_ROBOT_HEADING_DEG);
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

		// Best is scored on resolved range, so height is accounted for.
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
			// publish() rejects null.
			orchestrator.publish("vision/best", best);
		}
	}

	// Null when the detection cannot be resolved.
	private ClusterTarget toClusterTarget(AprilTagDetection detection, double headingDeg, long nowNanos) {
		String name;
		String shortName;
		int percentFound;
		int tagsTotal = 0;

		if (detection instanceof AprilTagClusterDetection) {
			// Cluster detections always carry metadata: the SDK could not have matched tags to a
			// cluster without it, so there is no null check to do here.
			AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
			name = cluster.metadata.name;
			shortName = cluster.metadata.shortName;
			percentFound = cluster.percentClusterFound;
			tagsTotal = TAGS_PER_CLUSTER;
		} else if (detection instanceof AprilTagSingleDetection) {
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

		boolean allianceMatch = name.startsWith(String.valueOf(allianceLetter));

		// Turret yaw is folded in here, not in the SDK pose: resolved includes it, rawX/Y/Z do not.
		TargetingMath.ResolvedPoint resolvedPoint = TargetingMath.resolve(
				rawX, rawY, rawZ, rawYaw, cameraPose.getYaw(), cameraPose.getElevation(), headingDeg, poi);

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
				resolvedPoint,
				detection.frameAcquisitionNanoTime,
				ageMs,
				stale);
	}

	/** Puts the target worth acting on first, then a line for anything else. Verbose adds the
	 * configuration block and the SDK's raw numbers back. */
	public void publishTelemetry(Telemetry telemetry) {
		ClusterTarget best = lastBest;

		// The number you are driving by stays on top, so toggling verbose can never push it off screen.
		if (best == null) {
			telemetry.addLine("no usable cluster");
			for (ClusterTarget target : lastTargets) {
				telemetry.addLine("  " + target.compactLine());
			}
		} else {
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
			telemetry.addLine(String.format(
					Locale.US,
					"cfg %s %dx%d @%.0ffps decim %d  poi %s",
					STREAM_FORMAT,
					STREAM_WIDTH,
					STREAM_HEIGHT,
					getActualFps(),
					lastDecimation,
					poi.isEnabled()
							? poi.toString()
							: "off"));
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
					exposureLocked
							? "manual"
							: "auto",
					whiteBalanceLocked
							? "manual"
							: "auto",
					allianceLetter));
			telemetry.addLine(String.format(
					Locale.US,
					"head %+.1f%s   camera %s",
					lastHeadingDeg,
					lastHeadingDeg == DEFAULT_ROBOT_HEADING_DEG
							? " ASSUMED"
							: "",
					cameraName));
		}
	}

	/** Show the configuration block and raw SDK numbers. Off by default. */
	public void setVerbose(boolean verbose) {
		this.verbose = verbose;
	}

	/**
	 * Autofocus is pinned because it changes the focal length, and range scales with focal length.
	 * Every control is optional: each is tried independently and failures are recorded, not thrown.
	 */
	private void lockCameraControls() {
		// ---- focus: the one that matters for range ----
		try {
			FocusControl focus = visionPortal.getCameraControl(FocusControl.class);
			if (focus != null) {
				for (FocusControl.Mode mode : new FocusControl.Mode[] {FocusControl.Mode.Fixed, FocusControl.Mode.Infinity}) {
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
		// Called inline, not on the hardware thread: these are camera-control calls, not Lynx bus
		// traffic, and SafeOpMode's hardware thread does not tick until after this constructor.
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
			// MANUAL is upper case here, unlike ExposureControl and FocusControl, and this control
			// has no isModeSupported(), so setMode's return value is the only signal.
			if (balance != null && balance.setMode(WhiteBalanceControl.Mode.MANUAL)) {
				this.whiteBalanceLocked = true;
			}
		} catch (Exception e) {
			orchestrator.warn("White balance control unavailable: " + e.getMessage());
		}
	}

	/** Scaled to the actual frame interval: a fixed limit would flag every detection stale while the
	 * camera runs slowly, stranding the driver station on "no usable cluster". */
	private double staleThresholdMs() {
		float fps = getActualFps();
		if (fps <= 0.0f) {
			return STALE_AFTER_MS;
		}
		double frameIntervalMs = 1000.0 / fps;
		return Math.max(STALE_AFTER_MS, STALE_FRAME_INTERVALS * frameIntervalMs);
	}

	/** Frame rate the camera actually achieved; SDK 12.0 exposes no way to request one. */
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

	/** Change decimation live; higher decimation raises the frame rate but shortens detection range. */
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

	/** Adds to the turret yaw, in degrees, without a redeploy. */
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

	/** Restore default extrinsics; yaw takes effect immediately, the rest needs a restart. */
	@SubscribedTo(topic = "vision/resetCameraPose")
	public void onResetCameraPose(Boolean trigger) {
		if (Boolean.TRUE.equals(trigger)) {
			cameraPose.resetToDefaults();
		}
	}

	@Override
	public void close() {
		visionPortal.close();
	}
}
