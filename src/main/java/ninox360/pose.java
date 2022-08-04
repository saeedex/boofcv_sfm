package ninox360;

import boofcv.factory.geo.*;
import boofcv.struct.geo.AssociatedPair;
import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ejml.data.DMatrixRMaj;

import java.util.List;

public class pose {
    public static DMatrixRMaj robustFundamental(List<AssociatedPair> matches, List<AssociatedPair> inliers, double inlierThreshold ) {
        var configRansac = new ConfigRansac();
        configRansac.inlierThreshold = inlierThreshold;
        configRansac.iterations = 1000;
        ConfigFundamental configFundamental = new ConfigFundamental();
        configFundamental.which = EnumFundamental.LINEAR_7;
        configFundamental.numResolve = 2;
        configFundamental.errorModel = ConfigFundamental.ErrorModel.GEOMETRIC;
        // geometric error is the most accurate error metric, but also the slowest to compute. See how the
        // results change if you switch to sampson and how much faster it is. You also should adjust
        // the inlier threshold.

        ModelMatcher<DMatrixRMaj, AssociatedPair> ransac = FactoryMultiViewRobust.fundamentalRansac(configFundamental, configRansac);

        // Estimate the fundamental matrix while removing outliers
        if (!ransac.process(matches))
            throw new IllegalArgumentException("Failed");

        // save the set of features that were used to compute the fundamental matrix
        inliers.addAll(ransac.getMatchSet());

        // Improve the estimate of the fundamental matrix using non-linear optimization
        var F = new DMatrixRMaj(3, 3);
        ModelFitter<DMatrixRMaj, AssociatedPair> refine =
                FactoryMultiView.fundamentalRefine(1e-8, 400, EpipolarError.SAMPSON);
        if (!refine.fitModel(inliers, ransac.getModelParameters(), F))
            throw new IllegalArgumentException("Failed");

        // Return the solution
        return F;
    }
}
