package ninox360;
import boofcv.abst.geo.Estimate1ofEpipolar;
import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.abst.geo.Triangulate2ViewsMetricH;
import boofcv.alg.geo.DecomposeEssential;
import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.robust.Se3FromEssentialGenerator;
import boofcv.alg.geo.robust.SelectBestStereoTransform;
import boofcv.factory.geo.*;
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
        Se3_F64 model = new Se3_F64();

        // Method 1
        List<AssociatedPair> matches = new ArrayList<>();
        for (Track track : tracks) {
            var p = new AssociatedPair(cameras.get(track.camids.get(0)).kps.get(track.kpids.get(0)),
                    cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)));
            cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)).print();
            matches.add(p);
        }

        List<AssociatedPair> inliers = new ArrayList<>();
        DMatrixRMaj F = robustFundamental(matches, inliers, config);
        DMatrixRMaj E = fund2essential(F, config.K, config.K.copy());
        //List<Se3_F64> poses = MultiViewOps.decomposeEssential(E);
        E.print();

        // Method 2
        DMatrixRMaj Kinv = config.K.copy();

        /*
        DecomposeEssential decomposeE = new DecomposeEssential();
        Triangulate2ViewsMetric triangulate;
        SelectBestStereoTransform selectBest = new SelectBestStereoTransform(triangulate);
        decomposeE.decompose(E);
        List<Se3_F64> poses = decomposeE.getSolutions();
        selectBest.select(decomposeE.getSolutions(),inliers,model);
        */

        /*
        Estimate1ofEpipolar essentialAlg = FactoryMultiView.fundamental_1(EnumFundamental.LINEAR_8, 0);
        Triangulate2ViewsMetricH triangulate = FactoryMultiView.triangulate2ViewMetricH(new ConfigTriangulation(ConfigTriangulation.Type.GEOMETRIC));
        Se3FromEssentialGenerator alg = new Se3FromEssentialGenerator(essentialAlg, triangulate);

        alg.generate(matches, model);
        pose.R.print();
        */
        config.init = true;
        return model;
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
