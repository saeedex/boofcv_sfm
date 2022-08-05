package ninox360;
import boofcv.abst.geo.Estimate1ofEpipolar;
import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.abst.geo.Triangulate2ViewsMetricH;
import boofcv.alg.geo.DecomposeEssential;
import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.robust.ModelMatcherMultiview;
import boofcv.alg.geo.robust.Se3FromEssentialGenerator;
import boofcv.alg.geo.robust.SelectBestStereoTransform;
import boofcv.factory.distort.LensDistortionFactory;
import boofcv.factory.geo.*;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.GeoModelEstimator1;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.ops.CommonOps_BDRM;

import java.util.ArrayList;
import java.util.List;



public class pose {
    public static Se3_F64 init(List<Track> tracks, List<Camera> cameras, Config config){
        List<AssociatedPair> matches = new ArrayList<>();
        for (Track track : tracks) {
            var p = new AssociatedPair(cameras.get(track.camids.get(0)).kps.get(track.kpids.get(0)),
                    cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)));
            matches.add(p);
        }

        List<AssociatedPair> matchedCalibrated = convertToNormalizedCoordinates(matches, config.intrinsic);
        List<AssociatedPair> inliers = new ArrayList<>();
        Se3_F64 pose = estimateCameraMotion(config.intrinsic, matchedCalibrated, inliers, config);
        System.out.println(pose);
        config.init = true;
        return pose;
    }
    public static Se3_F64 estimateCameraMotion(CameraPinholeBrown intrinsic, List<AssociatedPair> matchedNorm, List<AssociatedPair> inliers, Config config) {
        config.epiMotion.setIntrinsic(0, intrinsic);
        config.epiMotion.setIntrinsic(1, intrinsic);

        if (!config.epiMotion.process(matchedNorm))
            throw new RuntimeException("Motion estimation failed");

        // save inlier set for debugging purposes
        inliers.addAll(config.epiMotion.getMatchSet());

        return config.epiMotion.getModelParameters();
    }
    public static List<AssociatedPair> convertToNormalizedCoordinates( List<AssociatedPair> matchedFeatures, CameraPinholeBrown intrinsic ) {

        Point2Transform2_F64 p_to_n = LensDistortionFactory.narrow(intrinsic).undistort_F64(true, false);

        List<AssociatedPair> calibratedFeatures = new ArrayList<>();

        for (AssociatedPair p : matchedFeatures) {
            AssociatedPair c = new AssociatedPair();

            p_to_n.compute(p.p1.x, p.p1.y, c.p1);
            p_to_n.compute(p.p2.x, p.p2.y, c.p2);

            calibratedFeatures.add(c);
        }

        return calibratedFeatures;
    }
    public static DMatrixRMaj fund2essential(DMatrixRMaj F, DMatrixRMaj K1, DMatrixRMaj K2){
        CommonOps_DDRM.transpose(K1);
        DMatrixRMaj Et = F.copy();
        DMatrixRMaj E = Et.copy();
        CommonOps_DDRM.mult(K1, F, Et);
        CommonOps_DDRM.mult(Et, K2, E);
        return E;
    }
    public static DMatrixRMaj robustFundamental(List<AssociatedPair> matches, List<AssociatedPair> inliers, Config config) {
        // Estimate the fundamental matrix while removing outliers
        if (!config.funRansac.process(matches))
            throw new IllegalArgumentException("Failed");

        // save the set of features that were used to compute the fundamental matrix
        inliers.addAll(config.funRansac.getMatchSet());

        // Improve the estimate of the fundamental matrix using non-linear optimization
        var F = new DMatrixRMaj(3, 3);
        if (!config.refine.fitModel(inliers, config.funRansac.getModelParameters(), F))
            throw new IllegalArgumentException("Failed");

        MultiViewOps.decomposeEssential(F);
        // Return the solution
        return F;
    }

    public static DMatrixRMaj robustEssential(List<AssociatedPair> matches, List<AssociatedPair> inliers, Config config) {
        // Estimate the essential matrix while removing outliers
        if (!config.essRansac.process(matches))
            throw new IllegalArgumentException("Failed");

        // save the set of features that were used to compute the fundamental matrix
        inliers.addAll(config.essRansac.getMatchSet());

        // Improve the estimate of the fundamental matrix using non-linear optimization
        var F = new DMatrixRMaj(3, 3);
        if (!config.refine.fitModel(inliers, config.essRansac.getModelParameters(), F))
            throw new IllegalArgumentException("Failed");

        MultiViewOps.decomposeEssential(F);
        // Return the solution
        return F;
    }
}
