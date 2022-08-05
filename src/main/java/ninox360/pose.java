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
import boofcv.gui.feature.AssociationPanel;
import boofcv.gui.image.ShowImages;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.GeoModelEstimator1;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.ops.CommonOps_BDRM;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;



public class pose {
    public static Se3_F64 init(List<Track> tracks, List<Camera> cameras, Config config){
//        Se3_F64 model = new Se3_F64();

        // Method 1
        List<AssociatedPair> matches = new ArrayList<>();
        for (Track track : tracks) {
            var p = new AssociatedPair(cameras.get(track.camids.get(0)).kps.get(track.kpids.get(0)),
                    cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)));
            cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)).print();
            matches.add(p);
        }
        CameraPinholeBrown intrinsic = new CameraPinholeBrown();
        intrinsic.fsetK(config.K);
        List<AssociatedPair> matchedCalibrated = convertToNormalizedCoordinates(matches, intrinsic);
        List<AssociatedPair> inliers = new ArrayList<>();
        Se3_F64 pose = estimateCameraMotion(intrinsic, matchedCalibrated, inliers);
        System.out.println(pose);
//        drawInliers(cameras.get(1).img, cameras.get(0).img, intrinsic, inliers);
        return pose;
    }
    public static Se3_F64 estimateCameraMotion( CameraPinholeBrown intrinsic,
                                                List<AssociatedPair> matchedNorm, List<AssociatedPair> inliers ) {
        ModelMatcherMultiview<Se3_F64, AssociatedPair> epipolarMotion =
                FactoryMultiViewRobust.baselineRansac(new ConfigEssential(), new ConfigRansac(200, 0.5));
        epipolarMotion.setIntrinsic(0, intrinsic);
        epipolarMotion.setIntrinsic(1, intrinsic);

        if (!epipolarMotion.process(matchedNorm))
            throw new RuntimeException("Motion estimation failed");

        // save inlier set for debugging purposes
        inliers.addAll(epipolarMotion.getMatchSet());

        return epipolarMotion.getModelParameters();
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
    public static void drawInliers(BufferedImage left, BufferedImage right, CameraPinholeBrown intrinsic,
                                   List<AssociatedPair> normalized ) {
        Point2Transform2_F64 n_to_p = LensDistortionFactory.narrow(intrinsic).distort_F64(false, true);

        List<AssociatedPair> pixels = new ArrayList<>();

        for (AssociatedPair n : normalized) {
            AssociatedPair p = new AssociatedPair();

            n_to_p.compute(n.p1.x, n.p1.y, p.p1);
            n_to_p.compute(n.p2.x, n.p2.y, p.p2);

            pixels.add(p);
        }

        // display the results
        AssociationPanel panel = new AssociationPanel(20);
        panel.setAssociation(pixels);
        panel.setImages(left, right);

        ShowImages.showWindow(panel, "Inlier Features", true);
    }


}
