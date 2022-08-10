package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.extract.ConfigExtract;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.geo.Estimate1ofPnP;
import boofcv.abst.geo.RefinePnP;
import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.abst.geo.TriangulateNViewsMetric;
import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.WorldToCameraToPixel;
import boofcv.alg.geo.robust.ModelMatcherMultiview;
import boofcv.factory.distort.LensDistortionFactory;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.*;
import boofcv.gui.ListDisplayPanel;
import boofcv.misc.ConfigConverge;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayF32;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.TwoAxisRgbPlane;
import boofcv.visualize.VisualizeData;
import georegression.metric.UtilAngle;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ejml.data.DMatrixRMaj;

import java.awt.*;
import java.awt.image.BufferedImage;

public class Config {
    DetectDescribePoint<GrayF32, TupleDesc_F64> describer;
    ScoreAssociation<TupleDesc_F64> scorer;
    AssociateDescription<TupleDesc_F64> matcher;

    ConfigRansac ransac;
    ModelMatcherMultiview<Se3_F64, AssociatedPair> epiMotion;

    ModelMatcherMultiview<Se3_F64, Point2D3D> estimatePnP;

    RefinePnP refinePnP;

    TriangulateNViewsMetric trian;
    CameraPinholeBrown intrinsic;
    DMatrixRMaj K;
    Point2Transform2_F64 norm;

    ScaleSceneStructure bundleScale;
    BundleAdjustment<SceneStructureMetric> bundleAdjustment;

    boolean init = false;

    PointCloudViewer viewer;
    ListDisplayPanel gui;

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
        ConfigEssential configEssential = new ConfigEssential();
        configEssential.which = EnumEssential.LINEAR_8;
        configEssential.numResolve = 2;
        configEssential.errorModel = ConfigEssential.ErrorModel.GEOMETRIC;

        this.epiMotion = FactoryMultiViewRobust.baselineRansac(configEssential, configRansac);
        this.estimatePnP = FactoryMultiViewRobust.pnpRansac(new ConfigPnP(), configRansac);
        this.refinePnP = FactoryMultiView.pnpRefine(1e-12,40);

        // triangulation
        // ConfigTriangulation.Type.GEOMETRIC resulted in better triangulation accuracy
        this.trian = FactoryMultiView.triangulateNViewMetric(new ConfigTriangulation(ConfigTriangulation.Type.GEOMETRIC));

        // Bundle adjustment
        var configLM = new ConfigLevenbergMarquardt();
        configLM.dampeningInitial = 1e-3;
        configLM.hessianScaling = true;
        var configSBA = new ConfigBundleAdjustment();
        configSBA.configOptimizer = configLM;
        this.bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);
        this.bundleAdjustment.setVerbose(System.out, null);
        this.bundleAdjustment.configure(1e-6, 1e-6, 50);
        this.bundleScale = new ScaleSceneStructure();

        // Viewer
        this.viewer = VisualizeData.createPointCloudViewer();
        this.viewer.setCameraHFov(UtilAngle.radian(60));
        this.viewer.getComponent().setPreferredSize(new Dimension(600, 600));
        this.viewer.setFog(true);
        this.viewer.setColorizer(new TwoAxisRgbPlane.Z_XY(1.0).fperiod(40));
        this.viewer.setDotSize(1);
        this.gui = new ListDisplayPanel();
    }

    public void getIntrinsic(BufferedImage sample){
        int height = sample.getHeight();
        int width = sample.getWidth();
        this.intrinsic = new CameraPinholeBrown(height, height, 0, width/2, height/2, width, height);
        this.K = PerspectiveOps.pinholeToMatrix(intrinsic, (DMatrixRMaj)null);
        this.norm = LensDistortionFactory.narrow(this.intrinsic).undistort_F64(true, false);
    }

}