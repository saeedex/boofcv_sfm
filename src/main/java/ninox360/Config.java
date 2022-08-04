package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.extract.ConfigExtract;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;

public class Config {
    DetectDescribePoint<GrayF32, TupleDesc_F64> describer;
    ScoreAssociation<TupleDesc_F64> scorer;
    AssociateDescription<TupleDesc_F64> matcher;

    public Config(int numFeatures, double matcherThreshold){
        ConfigFastHessian configDetector = new ConfigFastHessian();
        configDetector.extract = new ConfigExtract(2, 0, 5, true);
        configDetector.maxFeaturesPerScale = numFeatures;
        configDetector.initialSampleStep = 2;
        this.describer = FactoryDetectDescribe.surfStable(configDetector, null, null,GrayF32.class);
        this.scorer = FactoryAssociation.scoreEuclidean(TupleDesc_F64.class, true);
        this.matcher = FactoryAssociation.greedy(new ConfigAssociateGreedy(true, matcherThreshold), this.scorer);
    }
}
