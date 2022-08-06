package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.extract.ConfigExtract;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.robust.ModelMatcherMultiview;
import boofcv.factory.distort.LensDistortionFactory;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.*;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ejml.data.DMatrixRMaj;
import java.awt.image.BufferedImage;

public class Config {
    DetectDescribePoint<GrayF32, TupleDesc_F64> describer;
    ScoreAssociation<TupleDesc_F64> scorer;
    AssociateDescription<TupleDesc_F64> matcher;

    ConfigRansac ransac;
    ModelMatcherMultiview<Se3_F64, AssociatedPair> epiMotion;

    ModelMatcher<DMatrixRMaj, AssociatedPair> funRansac;
    ModelMatcher<DMatrixRMaj, AssociatedPair> essRansac;
    ModelFitter<DMatrixRMaj, AssociatedPair> refine;
    Triangulate2ViewsMetric trian;
    CameraPinholeBrown intrinsic;
    DMatrixRMaj K;
    Point2Transform2_F64 norm;
    boolean init = false;

    public Config(int numFeatures, double matcherThreshold, double inlierThreshold){
        // describer and matcher
        ConfigFastHessian configDetector = new ConfigFastHessian();
        configDetector.extract = new ConfigExtract(2, 0, 5, true);
        configDetector.maxFeaturesPerScale = numFeatures;
        configDetector.initialSampleStep = 2;
        this.describer = FactoryDetectDescribe.surfStable(configDetector, null, null,GrayF32.class);
        this.scorer = FactoryAssociation.scoreEuclidean(TupleDesc_F64.class, true);
        this.matcher = FactoryAssociation.greedy(new ConfigAssociateGreedy(true, matcherThreshold), this.scorer);

        // ransac
        var configRansac = new ConfigRansac();
        configRansac.inlierThreshold = inlierThreshold;
        configRansac.iterations = 1000;

        // motion
        ConfigFundamental configFundamental = new ConfigFundamental();
        configFundamental.which = EnumFundamental.LINEAR_8;
        configFundamental.numResolve = 2;
        configFundamental.errorModel = ConfigFundamental.ErrorModel.GEOMETRIC;

        ConfigEssential configEssential = new ConfigEssential();
        configEssential.which = EnumEssential.LINEAR_8;
        configEssential.numResolve = 2;
        configEssential.errorModel = ConfigEssential.ErrorModel.GEOMETRIC;

        // triangulation
        this.trian = FactoryMultiView.triangulate2ViewMetric(new ConfigTriangulation());
        this.epiMotion = FactoryMultiViewRobust.baselineRansac(configEssential, configRansac);
        this.funRansac = FactoryMultiViewRobust.fundamentalRansac(configFundamental, configRansac);
        this.essRansac = FactoryMultiViewRobust.essentialRansac(configEssential, configRansac);
        this.refine = FactoryMultiView.fundamentalRefine(1e-8, 400, EpipolarError.SAMPSON);
    }

    public void setintrinsic(BufferedImage sample){
        int height = sample.getHeight();
        int width = sample.getWidth();
        this.intrinsic = new CameraPinholeBrown(height, height, 0, width/2, height/2, width, height);
        this.K = PerspectiveOps.pinholeToMatrix(intrinsic, (DMatrixRMaj)null);
        this.norm = LensDistortionFactory.narrow(this.intrinsic).undistort_F64(true, false);
    }
}