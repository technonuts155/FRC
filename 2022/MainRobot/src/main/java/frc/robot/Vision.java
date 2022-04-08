package frc.robot;

import java.lang.invoke.LambdaMetafactory;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class Vision {
    private static PhotonCamera hubCam = new PhotonCamera("HUB Cam");
    private static PhotonPipelineResult latestResult = null;
    private static double camHeight = Units.inchesToMeters(38.5);
    private static double camPitch = Units.degreesToRadians(55);
    private static double targetHeight = Units.inchesToMeters(106);

    public static void updatePipelineResult() {
        latestResult = hubCam.getLatestResult();
    }

    public static boolean hasTargets() {
        return latestResult == null ? false : latestResult.hasTargets();
    }

    public static PhotonTrackedTarget getBestTarget() {
        return latestResult == null ? null : latestResult.getBestTarget();
    }

    public static double distanceFromHub() {
        PhotonTrackedTarget target = getBestTarget();
        if (target == null)
            return 50;

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            camHeight,
            targetHeight,
            camPitch,
            Units.degreesToRadians(target.getPitch())
        );

        return Units.metersToFeet(distance) * 1.81;
    }

}
