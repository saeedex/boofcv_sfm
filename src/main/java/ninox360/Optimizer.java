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
import boofcv.misc.BoofMiscOps;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ejml.data.DMatrixRMaj;

import java.util.List;

final class Scene {
    private final SceneStructureMetric structure;
    private final SceneObservations observations;

    public Scene(SceneStructureMetric structure, SceneObservations observations) {
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
    Scene scene;
    ScaleSceneStructure bundleScale;
    BundleAdjustment<SceneStructureMetric> bundleAdjustment;

    public Optimizer(){
        var configLM = new ConfigLevenbergMarquardt();
        configLM.dampeningInitial = 1e-3;
        configLM.hessianScaling = true;
        var configSBA = new ConfigBundleAdjustment();
        configSBA.configOptimizer = configLM;
        this.bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);
        this.bundleAdjustment.setVerbose(null, null);
        this.bundleAdjustment.configure(1e-6, 1e-6, 50);
        this.bundleScale = new ScaleSceneStructure();

        SceneStructureMetric structure = new SceneStructureMetric(false);
        SceneObservations observations = new SceneObservations();
        this.scene = new Scene(structure, observations);
    }

    public void initScene(List<Track> tracks, List<View> views){
        SceneStructureMetric structure = this.scene.getStructure();
        SceneObservations observations = this.scene.getObservations();

        int cnt = 0;
        for (Track track: tracks) if (track.valid) cnt +=1;
        structure.initialize(1, views.size(), cnt);
        observations.initialize(views.size());
    }

    public void wrapScene(List<Track> tracks, List<View> views, Config config){
        SceneStructureMetric structure = this.scene.getStructure();
        SceneObservations observations = this.scene.getObservations();

        // set cameras
        structure.setCamera(0, false, config.intrinsic);

        // set views
        //structure.setView(0, 0, true, views.get(0).pose,-1);
        //for (int viewId = 1; viewId < views.size(); viewId++) {
        //    structure.setView(viewId, 0, false, views.get(viewId).conns.get(0).getMotion(), viewId-1);
        //}

        for (int viewId = 0; viewId < views.size(); viewId++) {
            int world_to_left_idx;
            if (viewId==0)
                world_to_left_idx = structure.addMotion(true, views.get(viewId).pose);
            else
                world_to_left_idx = structure.addMotion(false, views.get(viewId).pose);
            structure.setView(viewId, 0, world_to_left_idx, -1);
        }

        // set points
        int trackId = 0;
        for (Track track: tracks){
            if (track.valid){
                track.setValidId(trackId);
                structure.setPoint(track.validId, track.str.x, track.str.y, track.str.z);
                for (int i = 0; i < track.viewIds.size(); i++) {
                    int viewId = track.viewIds.get(i);
                    observations.views.get(viewId).add(track.validId,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).x,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).y);
                    structure.connectPointToView(track.validId, viewId);
                }
                trackId += 1;
            }
        }
    }

    public void unwrapScene(List<Track> tracks, List<View> views, Config config){

        // get cameras
        SceneStructureMetric structure = this.scene.getStructure();
        BundleAdjustmentOps.convert(((BundlePinholeBrown)structure.cameras.get(0).model),
                config.intrinsic.width, config.intrinsic.height, config.intrinsic);
        config.K = PerspectiveOps.pinholeToMatrix(config.intrinsic, (DMatrixRMaj)null);

        // get views
        //views.get(0).setPose(structure.getParentToView(0));
        //for (int viewId = 1; viewId < views.size(); viewId++) {
        //    views.get(viewId).conns.get(0).setMotion(structure.getParentToView(viewId));
        //}

        for (int viewId = 0; viewId < views.size(); viewId++) {
            views.get(viewId).setPose(structure.getParentToView(viewId));
        }

        // get points
        int trackId = 0;
        for (Track track: tracks){
            if (track.valid){
                structure.points.get(trackId).get(track.str);
                trackId += 1;
            }
        }
    }

    public void process(){
        SceneStructureMetric structure = this.scene.getStructure();
        SceneObservations observations = this.scene.getObservations();

        // bundle adjustment
        this.bundleScale.applyScale(structure, observations);
        this.bundleAdjustment.setParameters(structure, observations);
        long startTime = System.currentTimeMillis();
        double errorBefore = this.bundleAdjustment.getFitScore();
        if (!this.bundleAdjustment.optimize(structure)) {
            throw new RuntimeException("Bundle adjustment failed?!?");
        }
        System.out.println();
        System.out.printf("Error reduced by %.1f%%\n", (100.0*(errorBefore/this.bundleAdjustment.getFitScore() - 1.0)));
        System.out.println(BoofMiscOps.milliToHuman(System.currentTimeMillis() - startTime));
        this.bundleScale.undoScale(structure, observations);
    }
}
