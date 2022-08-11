package ninox360;

import boofcv.abst.geo.bundle.MetricBundleAdjustmentUtils;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.bundle.BundleAdjustmentOps;
import boofcv.alg.geo.bundle.cameras.BundlePinholeBrown;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.FastAccess;
import org.ejml.data.DMatrixRMaj;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

final class Connection {
    int viewId;
    FastAccess<AssociatedIndex> idxPair;
    Se3_F64 motion;
    public Connection(int viewId, FastAccess<AssociatedIndex> idxPair, Se3_F64 motion) {
        this.viewId = viewId;
        this.idxPair = idxPair;
        this.motion = motion;
    }
    public int getViewId(){return viewId;}
    public FastAccess<AssociatedIndex> getIdxPair(){return idxPair;}
    public Se3_F64 getMotion(){return motion;}
    public void setMotion(Se3_F64 motion){this.motion = motion;}
}

public class View {
    int id;
    String file;
    BufferedImage img;
    List<Point2D_F64> kps; // image coordinates
    List<Point2D_F64> obs = new ArrayList<>(); //normalized image coordinates
    List<Point2D_F64> prj = new ArrayList<>(); //projected image coordinates
    DogArray<TupleDesc_F64> dscs;
    List<Integer> trackIds;
    Se3_F64 pose = new Se3_F64();
    List<Connection> conns = new ArrayList<>();


    public View(int id, String file, Config config){
        BufferedImage img = UtilImageIO.loadImageNotNull(file);
        Feat feat = features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config);
        this.id = id;
        this.file = file;
        this.img = img;
        this.kps = feat.getkps();
        this.dscs = feat.getdscs();
        this.trackIds = feat.getTrackIds();
        this.normKps(config);
    }
    public void setPose(Se3_F64 pose){
        this.pose = pose;
    }

    public void addConnection(int matchViewId, DogArray<TupleDesc_F64> matchViewDscs, Config config){
        FastAccess<AssociatedIndex> idxPair = features.match(this.dscs, matchViewDscs, config);
        this.conns.add(new Connection(matchViewId, idxPair, new Se3_F64()));
    }
    public void mapTracks(List<Track> tracks, List<View> views) {
        for (Connection conn : this.conns) {
            int matchViewId = conn.viewId;
            View matchView = views.get(matchViewId);
            FastAccess<AssociatedIndex> idxPair = conn.idxPair;

            for (int i = 0; i < idxPair.size; i++) {
                int mtrkId = matchView.trackIds.get(idxPair.get(i).dst);

                if (mtrkId == -1) {
                    // Create new tracks
                    List<Integer> viewIds = new ArrayList<>();
                    List<Integer> kpIds = new ArrayList<>();
                    viewIds.add(matchViewId);
                    kpIds.add(idxPair.get(i).dst);
                    tracks.add(new Track(tracks.size(), 1, viewIds, kpIds));
                    int trkId = tracks.size() - 1;
                    matchView.trackIds.set(idxPair.get(i).dst, trkId);
                    mtrkId = trkId;
                }

                // Update existing tracks and structure
                tracks.get(mtrkId).viewIds.add(this.id);
                tracks.get(mtrkId).kpids.add(idxPair.get(i).src);
                tracks.get(mtrkId).length += 1;
                this.trackIds.set(idxPair.get(i).src, mtrkId);

                /*
                if (tracks.get(mtrkId).valid) {
                    observations.views.get(this.id).add(tracks.get(mtrkId).validId,
                            (float)views.get(this.id).kps.get(idxPair.get(i).src).x,
                            (float)views.get(this.id).kps.get(idxPair.get(i).src).y);
                    structure.connectPointToView(tracks.get(mtrkId).validId, this.id);
                }
                 */
            }
        }
    }
    public void triangulateTracks(List<Track> tracks, List<View> views, Config config){
        int cnt = 0;
        for (Track track: tracks){
            if (!track.valid) {
                track.triangulateN(views, config);
                if (track.valid) cnt += 1;
                /*
                if (track.valid) {
                    structure.points.grow();
                    track.setValidId(structure.points.size - 1);
                    structure.setPoint(track.validId, track.str.x, track.str.y, track.str.z);

                    for (int i = 0; i < track.viewIds.size(); i++) {
                        int viewId = track.viewIds.get(i);
                        observations.views.get(viewId).add(track.validId,
                                (float) views.get(viewId).kps.get(track.kpids.get(i)).x,
                                (float) views.get(viewId).kps.get(track.kpids.get(i)).y);
                        structure.connectPointToView(track.validId, viewId);
                    }
                }
                 */
            }
        }
    }
    public void normKps(Config config){
        Point2D_F64 pointNorm = new Point2D_F64();
        for (Point2D_F64 kp: this.kps){
            config.norm.compute(kp.x, kp.y, pointNorm);
            this.obs.add(pointNorm.copy());
        }
    }

    public void estimatePose(List<Track> tracks, List<View> views, Config config){
        int mid = this.conns.get(0).viewId;
        View matchView = views.get(mid);
        Se3_F64 motionAtoB = new Se3_F64();;
        Se3_F64 motionWorldToB = new Se3_F64();;

        if (!config.init) {
            // collect list of matches
            List<AssociatedPair> matches = new ArrayList<>();
            for (int i = 0; i < this.conns.get(0).idxPair.size; i++) {
                var p = new AssociatedPair(matchView.obs.get(this.conns.get(0).idxPair.get(i).dst),
                        this.obs.get(this.conns.get(0).idxPair.get(i).src));
                matches.add(p);
            }

            // estimate camera motion
            config.epiMotion.setIntrinsic(0, config.intrinsic);
            config.epiMotion.setIntrinsic(1, config.intrinsic);
            if (!config.epiMotion.process(matches))
                throw new RuntimeException("Motion estimation failed");
            motionAtoB = config.epiMotion.getModelParameters();
            motionWorldToB = motionAtoB.copy();

            config.init = true;
            //structure.setView(0, 0, true, views.get(0).pose, -1);
        }
        else {
            // collect list of matches
            List<Point2D3D> matches = new ArrayList<>();
            for (int i = 0; i < this.trackIds.size(); i++) {
                int trackId = this.trackIds.get(i);
                if (trackId != -1){
                    if (tracks.get(trackId).valid) {
                        matches.add(new Point2D3D(this.obs.get(i), tracks.get(trackId).str));
                    }
                }
            }

            // estimate camera motion
            config.estimatePnP.setIntrinsic(0, config.intrinsic);
            if( !config.estimatePnP.process(matches))
                throw new RuntimeException("Motion estimation failed");

            // refine the motion estimate using non-linear optimization
            if( !config.refinePnP.fitModel(config.estimatePnP.getMatchSet(), config.estimatePnP.getModelParameters(), motionWorldToB) )
                throw new RuntimeException("Refine failed!?!?");

            // compute relative motion of current camera for the connection
            Se3_F64 motionBtoWorld = motionWorldToB.invert(null);
            Se3_F64 motionWorldToA = matchView.pose;
            Se3_F64 motionBtoA =  motionBtoWorld.concat(motionWorldToA, null);
            motionAtoB = motionBtoA.invert(null);
        }

        // set camera pose and relative motion
        this.setPose(motionWorldToB);
        this.conns.get(0).setMotion(motionAtoB);
        //structure.setView(id, 0, true, views.get(id).conns.get(0).getMotion(), mid);
    }

    public void projectTracks(List<Track> tracks, Config config){
        for (int trackId : this.trackIds) {
            if (trackId != -1) {
                if (tracks.get(trackId).valid) {
                    Point2D_F64 kprj = tracks.get(trackId).project(this, config);
                    this.prj.add(kprj);
                }
            }
        }
    }
    public void viewTracks(List<Track> tracks) {
        Graphics2D vImg = this.img.createGraphics();
        for (int i = 0; i < this.kps.size(); i++) {
            if (this.trackIds.get(i) != -1) {
                if (tracks.get(this.trackIds.get(i)).valid) VisualizeFeatures.drawPoint(vImg,
                        this.kps.get(i).x, this.kps.get(i).y, 2, Color.BLUE, false);
            }
        }
    }
    public void viewProjections(){
        Graphics2D vImg = this.img.createGraphics();
        for (Point2D_F64 point2D_f64 : this.prj) {
            VisualizeFeatures.drawPoint(vImg, point2D_f64.x, point2D_f64.y, 2, Color.RED, false);
        }
    }

    public static void viewViews(List<Track> tracks, List<View> views, Config config){
        for (View view : views){
            view.projectTracks(tracks, config);
            view.viewTracks(tracks);
            view.viewProjections();
            config.gui.addImage(view.img, view.file);
        }
        ShowImages.showWindow(config.gui,"detected features", true);
    }
    public static void wrapScene(SceneStructureMetric structure, SceneObservations observations,
                                 List<Track> tracks, List<View> views, Config config){
        int cnt = 0;
        for (Track track: tracks)
            if (track.valid) cnt +=1;
        structure.initialize(1, views.size(), cnt);
        observations.initialize(views.size());

        // set cameras
        structure.setCamera(0, false, config.intrinsic);

        // set views
        /*
        structure.setView(0, 0, true, views.get(0).pose,-1);
        for (int viewId = 1; viewId < views.size(); viewId++) {
            structure.setView(viewId, 0, false, views.get(viewId).conns.get(0).getMotion(), viewId-1);
        }
         */
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

    public static void unwrapScene(SceneStructureMetric structure, SceneObservations observations,
                                 List<Track> tracks, List<View> views, Config config){
        // get cameras
        BundleAdjustmentOps.convert(((BundlePinholeBrown)structure.cameras.get(0).model),
                config.intrinsic.width, config.intrinsic.height, config.intrinsic);
        config.K = PerspectiveOps.pinholeToMatrix(config.intrinsic, (DMatrixRMaj)null);

        // get views
        /*
        views.get(0).setPose(structure.getParentToView(0));
        for (int viewId = 1; viewId < views.size(); viewId++) {
            views.get(viewId).conns.get(0).setMotion(structure.getParentToView(viewId));
        }
        */

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
    public static void bundleAdjustment(List<Track> tracks, List<View> views, Config config){
        // wrap structure and observations for Bundle adjustment
        SceneStructureMetric structure = new SceneStructureMetric(false);
        SceneObservations observations = new SceneObservations();
        View.wrapScene(structure, observations, tracks, views, config);

        // bundle adjustment
        config.bundleScale.applyScale(structure, observations);
        config.bundleAdjustment.setParameters(structure, observations);
        long startTime = System.currentTimeMillis();
        double errorBefore = config.bundleAdjustment.getFitScore();
        if (!config.bundleAdjustment.optimize(structure)) {
            throw new RuntimeException("Bundle adjustment failed?!?");
        }
        System.out.println();
        System.out.printf("Error reduced by %.1f%%\n", (100.0*(errorBefore/config.bundleAdjustment.getFitScore() - 1.0)));
        System.out.println(BoofMiscOps.milliToHuman(System.currentTimeMillis() - startTime));
        config.bundleScale.undoScale(structure, observations);

        // unwrap Bundle Adjustment results
        View.unwrapScene(structure, observations, tracks, views, config);
    }
}