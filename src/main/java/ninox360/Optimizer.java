package ninox360;

import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.bundle.BundleAdjustmentOps;
import boofcv.alg.geo.bundle.cameras.BundlePinholeBrown;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import georegression.struct.point.Point4D_F64;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ejml.data.DMatrixRMaj;

import java.util.ArrayList;
import java.util.List;

final class Graph {
    private final SceneStructureMetric structure;
    private final SceneObservations observations;

    public Graph(SceneStructureMetric structure, SceneObservations observations) {
        this.structure = structure;
        this.observations = observations;
    }
    public SceneStructureMetric getStructure() {
        return structure;
    }
    public SceneObservations getObservations() {
        return observations;
    }
}

public class Optimizer {
    Graph graph;
    ScaleSceneStructure bundleScale;
    BundleAdjustment<SceneStructureMetric> bundleAdjustment;
    List<Integer> viewIds;
    List<Integer> trackIds;
    Boolean local;

    public Optimizer(boolean local){
        var configLM = new ConfigLevenbergMarquardt();
        configLM.dampeningInitial = 1e-3;
        configLM.hessianScaling = true;
        var configSBA = new ConfigBundleAdjustment();
        configSBA.configOptimizer = configLM;
        this.bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);
        //this.bundleAdjustment = FactoryMultiView.bundleDenseMetric(true, null);

        this.bundleAdjustment.setVerbose(null, null);
        this.bundleAdjustment.configure(1e-12, 1e-12, 50);
        this.bundleScale = new ScaleSceneStructure();

        SceneStructureMetric structure = new SceneStructureMetric(false); //pay attention
        SceneObservations observations = new SceneObservations();
        this.graph = new Graph(structure, observations);
        this.local = local;
    }

    public void initGraph(List<Track> tracks, List<View> views){
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();

        int numCams = 1;
        int numViews = views.size();
        if (this.local) numViews = 6;
        int start = views.size() - numViews;
        if (start < 0) start = 0;

        // views included in optimization
        this.viewIds = new ArrayList<>();
        for (int i = start; i < views.size(); i++){
            this.viewIds.add(views.get(i).id);
        }
        numViews = this.viewIds.size();

        // tracks included in optimization
        boolean isLocal = false;
        this.trackIds = new ArrayList<>();
        for (Track track: tracks) {
            if (track.valid) {
                for (int viewId: track.viewIds) {
                    if (this.viewIds.contains(viewId)) {
                        isLocal = true;
                        break;
                    }
                }
                if (isLocal) this.trackIds.add(track.id);
            }
        }
        int numPoints = this.trackIds.size();
        structure.initialize(numCams, numViews, numViews, numPoints, 0);
        observations.initialize(numViews);
    }

    public void wrapGraph(List<Track> tracks, List<View> views, Config config){
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();

        // set cameras
        structure.setCamera(0, true, config.intrinsic);

        // set views
        structure.setView(0, 0, true, views.get(this.viewIds.get(0)).pose,-1);
        for (int i = 1; i < this.viewIds.size(); i++) {
            structure.setView(i, 0, false, views.get(this.viewIds.get(i)).pose, -1);
        }

        // set points
        for (int j = 0; j < this.trackIds.size(); j++){
            Track track = tracks.get(this.trackIds.get(j));
            structure.points.get(j).set(track.str.x, track.str.y, track.str.z);

            for (int i = 0; i < track.viewIds.size(); i++) {
                //System.out.println(track.viewIds);
                int viewId = track.viewIds.get(i);
                if (track.inliers.get(i) && this.viewIds.contains(viewId)) {
                    observations.views.get(this.viewIds.indexOf(viewId)).add(j,
                            (float) views.get(viewId).kps.get(track.kpids.get(i)).x,
                            (float) views.get(viewId).kps.get(track.kpids.get(i)).y);
                    structure.connectPointToView(j, this.viewIds.indexOf(viewId));
                }
            }

        }
    }

    public void unwrapGraph(List<Track> tracks, List<View> views, Config config){

        // get cameras
        SceneStructureMetric structure = this.graph.getStructure();
        BundleAdjustmentOps.convert(((BundlePinholeBrown)structure.cameras.get(0).model),
                config.intrinsic.width, config.intrinsic.height, config.intrinsic);
        config.K = PerspectiveOps.pinholeToMatrix(config.intrinsic, (DMatrixRMaj)null);

        // get views
        for (int i = 1; i < this.viewIds.size(); i++) {
            views.get(this.viewIds.get(i)).setPose(structure.getParentToView(i));
        }

        // get points
        for (int j = 0; j < this.trackIds.size(); j++){
            Track track = tracks.get(this.trackIds.get(j));
            if (track.valid){
                structure.points.get(j).get(track.str);
            }
        }
    }

    public void process(){
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();

        // bundle adjustment
        //this.bundleScale.applyScale(structure, observations);
        this.bundleAdjustment.setParameters(structure, observations);
        double errorBefore = this.bundleAdjustment.getFitScore()/structure.getObservationCount();
        if (!this.bundleAdjustment.optimize(structure)) {
            throw new RuntimeException("Bundle adjustment failed?!?");
        }
        double errorAfter = this.bundleAdjustment.getFitScore()/structure.getObservationCount();
        System.out.println(this.bundleAdjustment.getFitScore()/structure.getObservationCount());
        System.out.printf("Error before %.4f\n", errorBefore);
        System.out.printf("Error after by %.4f\n", errorAfter);
        System.out.printf("Error reduced by %.1f%%\n", (100.0*(errorAfter/errorBefore - 1.0)));
        //this.bundleScale.undoScale(structure, observations);
    }

    public void triangulateInvalidTracks(List<Track> tracks, List<View> views, Config config){
        for (Track track: tracks) {
            if (!track.valid) {
                track.triangulateN(views, config);
                track.filter(views, config);
            }
        }
    }

    public void filterTracks(List<Track> tracks, List<View> views, Config config){
        for (Track track: tracks) {
            track.filter(views, config);
        }
    }
}
