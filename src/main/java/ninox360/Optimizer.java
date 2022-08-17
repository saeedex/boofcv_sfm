package ninox360;

import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.cloud.PointCloudReader;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.bundle.BundleAdjustmentOps;
import boofcv.alg.geo.bundle.cameras.BundlePinholeBrown;
import boofcv.alg.mvs.ColorizeMultiViewStereoResults;
import boofcv.core.image.LookUpColorRgbFormats;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.io.image.LookUpImageFilesByIndex;
import boofcv.io.points.PointCloudIO;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.VisualizeData;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point4D_F64;
import lombok.Getter;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.DogArray_I32;
import org.ejml.data.DMatrixRMaj;

import java.awt.*;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Graph class: Specifies a scene graph for optimizing using {@link BundleAdjustment}.
 * It specifies the relationships between cameras, views, points and their corresponding observations in views.
 */
final class Graph {
    /**
     * Structure class specifies cameras, views and points. A view is a set of observed features from a specific
     * camera image. Views have an associated camera and specify the pose of the camera when the scene was viewed. Points
     * describe the scene's 3D structure.
     */
    @Getter final SceneStructureMetric structure;
    /**
     * Storage for feature observation in each view. Input for bundle adjustment.
     */
    @Getter private final SceneObservations observations;
    /**
     * Specifies view pose and 3D structure visualizer.
     */
    @Getter private final PointCloudViewer viewer;

    public Graph(SceneStructureMetric structure, SceneObservations observations) {
        this.structure = structure;
        this.observations = observations;

        this.viewer = VisualizeData.createPointCloudViewer();
        this.viewer.setCameraHFov(UtilAngle.radian(60));
        this.viewer.getComponent().setPreferredSize(new Dimension(600, 600));
        this.viewer.setFog(true);
        //this.viewer.setColorizer(new TwoAxisRgbPlane.Z_XY(1.0).fperiod(40));
        this.viewer.setDotSize(1);
    }

    /**
     * Visualizes 3D point cloud at {@link Track}
     *
     * @param imageFiles image file names corresponding to views in structure.
     */
    public void viewCloud(List<String> imageFiles) {
        Point4D_F64 world = new Point4D_F64();
        var imageLookup = new LookUpImageFilesByIndex(imageFiles);
        var colorize = new ColorizeMultiViewStereoResults<>(new LookUpColorRgbFormats.PL_U8(), imageLookup);

        DogArray_I32 rgb = DogArray_I32.zeros(structure.points.size);
        colorize.processScenePoints(structure,
                (viewIdx) -> viewIdx + "", // String encodes the image's index
                (pointIdx, r, g, b) -> rgb.set(pointIdx, (r << 16) | (g << 8) | b)); // Assign the RGB color

        // Convert the structure into regular 3D points from homogenous
        for (int i = 0; i < structure.points.size; i++) {
            structure.points.get(i).get(world);
            if (world.w == 0.0)
                viewer.addPoint(0, 0, Double.MAX_VALUE, 255);
            else
                viewer.addPoint(world.x / world.w, world.y / world.w, world.z / world.w, rgb.get(i));
        }
    }

    /**
     * Saves 3D point cloud at {@link Track}
     */
    public void saveCloud() throws IOException {
        var copy = new DogArray<>(Point3dRgbI_F64::new);
        viewer.copyCloud(copy);

        try (var out = new FileOutputStream("saved_cloud.ply")) {
            PointCloudIO.save3D(PointCloudIO.Format.PLY, PointCloudReader.wrapF64RGB(copy.toList()), true, out);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

/**
 * Specifies an optimization algorithm for Bundle Adjustment. Optimizes camera model, {@link View} poses and 3D structure at {@link Track}.
 */
public class Optimizer {
    /**
     * Scene graph specifying the relationships between cameras,
     * views, points and their corresponding observations in views.
     */
    Graph graph;
    /**
     * Normalizes variables in the graph to improve optimization performance.
     */
    ScaleSceneStructure bundleScale;
    /**
     * Interface for solving Bundle Adjustment problem.
     */
    BundleAdjustment<SceneStructureMetric> bundleAdjustment;
    /**
     * List of local {@link View} indices included in the optimization.
     */
    List<Integer> viewIds;
    /**
     * List of local {@link Track} indices included in the optimization.
     */
    List<Integer> trackIds;
    /**
     * A flag indicating if optimization is performed locally or globally. true for local optimization
     */
    Boolean local;

    public Optimizer(boolean local) {
        var configLM = new ConfigLevenbergMarquardt();
        configLM.dampeningInitial = 1e-3;
        configLM.hessianScaling = true;
        var configSBA = new ConfigBundleAdjustment();
        configSBA.configOptimizer = configLM;
        this.bundleAdjustment = FactoryMultiView.bundleSparseMetric(configSBA);
        this.bundleAdjustment.setVerbose(null, null);
        this.bundleAdjustment.configure(1e-12, 1e-12, 50);
        this.bundleScale = new ScaleSceneStructure();

        SceneStructureMetric structure = new SceneStructureMetric(true); //pay attention
        SceneObservations observations = new SceneObservations();
        this.graph = new Graph(structure, observations);
        this.local = local;
    }

    /**
     * initializes scene graph.
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     */
    public void initGraph(List<Track> tracks, List<View> views) {
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();

        int numCams = 1;
        int numViews = views.size();
        if (this.local) numViews = 6;
        int start = views.size() - numViews;
        if (start < 0) start = 0;

        // views included in optimization
        this.viewIds = new ArrayList<>();
        for (int i = start; i < views.size(); i++) {
            this.viewIds.add(views.get(i).id);
        }
        numViews = this.viewIds.size();

        // tracks included in optimization
        this.trackIds = new ArrayList<>();
        for (Track track : tracks) {
            if (!track.valid)
                continue;

            // See if contains a local view
            for (int trackViewIdx = 0; trackViewIdx < track.viewIds.size; trackViewIdx++) {
                int viewId = track.viewIds.get(trackViewIdx);
                if (this.viewIds.contains(viewId)) {
                    this.trackIds.add(track.id);
                    break;
                }
            }
        }
        int numPoints = this.trackIds.size();
        structure.initialize(numCams, numViews, numViews, numPoints, 0);
        observations.initialize(numViews);
    }

    /**
     * wraps local tracks and views to scene graph.
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public void wrapGraph(List<Track> tracks, List<View> views, Config config) {
        // set cameras
        this.graph.getStructure().setCamera(0, true, config.intrinsic);
        // wrap local views
        wrapViews(views);
        // wrap local tracks
        wrapPoints(tracks, views);
    }

    /**
     * unwraps local tracks and views back from scene graph.
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */

    public void unwrapGraph(List<Track> tracks, List<View> views, Config config) {
        // get cameras
        SceneStructureMetric structure = this.graph.getStructure();
        BundleAdjustmentOps.convert(((BundlePinholeBrown)structure.cameras.get(0).model),
                config.intrinsic.width, config.intrinsic.height, config.intrinsic);
        config.K = PerspectiveOps.pinholeToMatrix(config.intrinsic, (DMatrixRMaj)null);
        // unwrap local views
        unwrapViews(views);
        // unwrap points to local tracks
        unwrapPoints(tracks);
        // validate tracks
        filterTracks(tracks, views, config);
        triangulateInvalidTracks(tracks, views, config);
        filterTracks(tracks, views, config);
    }

    /**
     * performs optimization
     */
    public void process() {
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();

        // bundle adjustment
        //this.bundleScale.applyScale(structure, observations);
        this.bundleAdjustment.setParameters(structure, observations);
        double errorBefore = this.bundleAdjustment.getFitScore() / structure.getObservationCount();
        if (!this.bundleAdjustment.optimize(structure)) {
            throw new RuntimeException("Bundle adjustment failed?!?");
        }
        double errorAfter = this.bundleAdjustment.getFitScore() / structure.getObservationCount();
        System.out.printf("      Error reduced by %.1f%%\n", (100.0 * (errorAfter / errorBefore - 1.0)));
        //this.bundleScale.undoScale(structure, observations);
    }

    /**
     * Wraps local views
     *
     * @param views list of {@link View}
     */
    public void wrapViews(List<View> views) {
        SceneStructureMetric structure = this.graph.getStructure();
        structure.setView(0, 0, true, views.get(this.viewIds.get(0)).motionWorldToView, -1);
        for (int i = 1; i < this.viewIds.size(); i++) {
            structure.setView(i, 0, false, views.get(this.viewIds.get(i)).motionWorldToView, -1);
        }
    }

    /**
     * Unwraps local views
     *
     * @param views list of {@link View}
     */
    public void unwrapViews(List<View> views) {
        SceneStructureMetric structure = this.graph.getStructure();
        for (int i = 1; i < this.viewIds.size(); i++) {
            views.get(this.viewIds.get(i)).motionWorldToView.setTo(structure.getParentToView(i));
        }
    }

    /**
     * Wraps local tracks
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     */
    public void wrapPoints(List<Track> tracks, List<View> views) {
        SceneStructureMetric structure = this.graph.getStructure();
        SceneObservations observations = this.graph.getObservations();
        for (int j = 0; j < this.trackIds.size(); j++) {
            Track track = tracks.get(this.trackIds.get(j));
            structure.points.get(j).set(track.str.x, track.str.y, track.str.z, 1.0);

            for (int i = 0; i < track.viewIds.size(); i++) {
                //System.out.println(track.viewIds);
                int viewId = track.viewIds.get(i);
                if (track.inliers.get(i) && this.viewIds.contains(viewId)) {
                    observations.views.get(this.viewIds.indexOf(viewId)).add(j,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).x,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).y);
                    structure.connectPointToView(j, this.viewIds.indexOf(viewId));
                }
            }
        }
    }

    /**
     * Unwraps local tracks
     *
     * @param tracks list of {@link Track}
     */
    public void unwrapPoints(List<Track> tracks) {
        SceneStructureMetric structure = this.graph.getStructure();
        for (int j = 0; j < this.trackIds.size(); j++) {
            Track track = tracks.get(this.trackIds.get(j));
            Point4D_F64 world = new Point4D_F64();
            structure.points.get(j).get(world);
            track.str.x = world.x / world.w;
            track.str.y = world.y / world.w;
            track.str.z = world.z / world.w;
        }
    }

    /**
     * Filters local tracks
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public void filterTracks(List<Track> tracks, List<View> views, Config config) {
        trackIds.forEach(trackId -> {
            tracks.get(trackId).filter(views, config);
        });
    }

    /**
     * Tries to retriangulate invalid local tracks.
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public void triangulateInvalidTracks(List<Track> tracks, List<View> views, Config config) {
        trackIds.forEach(trackId -> {
            if (!tracks.get(trackId).valid) {
                tracks.get(trackId).filter(views, config);
            }
        });
    }
}
