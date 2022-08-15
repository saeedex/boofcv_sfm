package ninox360;

import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.extract.ConfigExtract;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.geo.RefinePnP;
import boofcv.abst.geo.TriangulateNViewsMetric;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.robust.ModelMatcherMultiview;
import boofcv.factory.distort.LensDistortionFactory;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.*;
import boofcv.gui.ListDisplayPanel;
import boofcv.io.calibration.CalibrationIO;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayF32;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ejml.data.DMatrixRMaj;

import java.awt.image.BufferedImage;
import java.io.File;

/**
 * Config class sets system configurations for feature detection and matching,
 * camera intrinsics and extrinsics, triangulation and view visualization.
 */
public class Config {
    double geoThreshold;
    /**
     * feature detection and matching
     */
    double matcherThreshold;
    DetectDescribePoint<GrayF32, TupleDesc_F64> describer;
    ScoreAssociation<TupleDesc_F64> scorer;
    /**
     * camera model
     */
    CameraPinholeBrown intrinsic;
    DMatrixRMaj K;
    Point2Transform2_F64 norm;
    /**
     * camera motion
     */
    ModelMatcher<DMatrixRMaj, AssociatedPair> essRansac;
    ModelMatcherMultiview<Se3_F64, AssociatedPair> epiMotion;
    ModelMatcherMultiview<Se3_F64, Point2D3D> estimatePnP;
    RefinePnP refinePnP;
    /**
     * triangulation
     */
    TriangulateNViewsMetric trian;
    /**
     * visualization
     */
    ListDisplayPanel gui;

    /**
     * initialize the configuration
     * @param numFeatures maximum number of features to detect per view
     * @param matcherThreshold ratio threshold for feature matching [0-1]
     * @param inlierThreshold geometric threshold for pose estimation/triangulation/reprojection [0-1]
     */
    public Config(int numFeatures, double matcherThreshold, double inlierThreshold){
        // describer and matcher
        ConfigFastHessian configDetector = new ConfigFastHessian();
        configDetector.extract = new ConfigExtract(2, 0, 5, true);

        // detect fixed number of features, stable performance
        configDetector.maxFeaturesAll = numFeatures;
//        configDetector.maxFeaturesPerScale = numFeatures;
        configDetector.initialSampleStep = 2;

        this.matcherThreshold = matcherThreshold;
        this.describer = FactoryDetectDescribe.surfStable(configDetector, null, null,GrayF32.class);
        this.scorer = FactoryAssociation.scoreEuclidean(TupleDesc_F64.class, true);

        // ransac
        this.geoThreshold = inlierThreshold;
        var configRansac = new ConfigRansac();
        configRansac.inlierThreshold = inlierThreshold;
        configRansac.iterations = 600;

        // motion
        ConfigEssential configEssential = new ConfigEssential();
        configEssential.which = EnumEssential.LINEAR_8;
        configEssential.numResolve = 2;
        configEssential.errorModel = ConfigEssential.ErrorModel.GEOMETRIC;

        this.essRansac = FactoryMultiViewRobust.essentialRansac(configEssential, configRansac);
        this.epiMotion = FactoryMultiViewRobust.baselineRansac(configEssential, configRansac);
        this.estimatePnP = FactoryMultiViewRobust.pnpRansac(new ConfigPnP(), configRansac);
        this.refinePnP = FactoryMultiView.pnpRefine(1e-12,40);

        // triangulation
        this.trian = FactoryMultiView.triangulateNViewMetric(new ConfigTriangulation(ConfigTriangulation.Type.GEOMETRIC));

        // Viewer
        this.gui = new ListDisplayPanel();
    }

    /**
     * Open an image and assume the camera's FOV is about 60 degrees
     */
    public void guessIntrinsics(BufferedImage sample){
        int height = sample.getHeight();
        int width = sample.getWidth();

        this.intrinsic = new CameraPinholeBrown(width, width, 0, width/2, height/2, width, height);
        this.K = PerspectiveOps.pinholeToMatrix(intrinsic, (DMatrixRMaj)null);
        this.norm = LensDistortionFactory.narrow(this.intrinsic).undistort_F64(true, false);
    }

    /**
     * load camera intrinsic
     * @param imageDirectory directory containing the intrinsic.yaml
     * @return true if loaded
     */
    public boolean loadIntrinsic(String imageDirectory){
        File file = new File(imageDirectory,"intrinsic.yaml");
        boolean flag = false;
        if (file.exists()) {
            this.intrinsic = CalibrationIO.load(file);
            this.K = PerspectiveOps.pinholeToMatrix(intrinsic, (DMatrixRMaj)null);
            this.norm = LensDistortionFactory.narrow(this.intrinsic).undistort_F64(true, false);
            flag = true;
        }
        return flag;
    }

}