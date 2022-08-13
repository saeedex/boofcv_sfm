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
import org.ddogleg.struct.FastArray;
import org.ejml.data.DMatrixRMaj;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

final class Connection {
    int viewId;
    FastAccess<AssociatedIndex> idxPair;
    private Se3_F64 motion;
    private double weight;
    List<AssociatedIndex> inlierPair;

    public Connection(int viewId, FastAccess<AssociatedIndex> idxPair) {
        this.viewId = viewId;
        this.idxPair = idxPair;
    }
    public void setWeight(double weight){this.weight = weight;}
    public void setMotion(Se3_F64 motion){this.motion = motion;}
    public double getWeight(){return this.weight;}

    public Se3_F64 getMotion(){return motion;}
    public double estimateMotion(int viewId, List<Track> tracks, List<View> views, Config config){
        View matchView = views.get(this.viewId);
        View view = views.get(viewId);
        Se3_F64 motionAtoB;
        Se3_F64 motionWorldToB = new Se3_F64();
        double weight;
        // if no valid tracks available initialize pose
        if (matchView.numOfTracks(tracks)==0) {
            List<AssociatedPair> matches = view.get2Dmatches(matchView, this);
            List<AssociatedPair> inliers = new ArrayList<>();
            // estimate camera motion
            config.epiMotion.setIntrinsic(0, config.intrinsic);
            config.epiMotion.setIntrinsic(1, config.intrinsic);
            if (!config.epiMotion.process(matches))
                throw new RuntimeException("Motion estimation failed");
            motionAtoB = config.epiMotion.getModelParameters();
            inliers.addAll(config.epiMotion.getMatchSet());
            weight = (double)inliers.size()/(double)matches.size();
        }
        // else use pnp to estimate pose
        else {
            List<Point2D3D> matches = view.get2D3Dmatches(tracks, matchView, this);
            List<Point2D3D> inliers = new ArrayList<>();
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
            inliers.addAll(config.estimatePnP.getMatchSet());
            weight = (double)inliers.size()/(double)matches.size();
            //System.out.println(inliers.size());
        }
        // set relative motion
        this.setMotion(motionAtoB);
        return weight;
    }
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

    public View(int id, String file, Config config, Feat feat){
        BufferedImage img = UtilImageIO.loadImageNotNull(file);
        //Feat feat = features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config);
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
    public void addConnections(List<Track> tracks, List<View> views, Config config, List<Integer> conviews){
        double numFeats = this.dscs.size();
        for (Integer matchViewId: conviews){
            FastAccess<AssociatedIndex> idxPair = features.match(this.dscs, views.get(matchViewId).dscs, config);
            double weight = idxPair.size/numFeats;

            // add previous view connection by default
            if (this.conns.size() == 0) {
                Connection conn = new Connection(matchViewId, idxPair);
                double newweight = conn.estimateMotion(this.id, tracks, views, config);
                conn.setWeight(newweight);
                this.conns.add(conn);
            }
            // more careful with loop closure
            // checking match ratio and geometric inliers before adding loop closure connection
            else if (weight > 0.4) {
                Connection conn = new Connection(matchViewId, idxPair);
                double newweight = conn.estimateMotion(this.id, tracks, views, config);
                conn.setWeight(newweight);
                if (conn.getWeight() > 0.5) this.conns.add(conn);
            }
        }
    }
    public void mapTracks(List<Track> tracks, List<View> views) {
        for (Connection conn : this.conns) {
            int matchViewId = conn.viewId;
            System.out.printf("  %6s: weight=%.2f\n", matchViewId, conn.getWeight());
            View matchView = views.get(matchViewId);
            FastAccess<AssociatedIndex> idxPair = conn.idxPair;

            for (int i = 0; i < idxPair.size(); i++) {
                int trkId = this.trackIds.get(idxPair.get(i).src);
                int mtrkId = matchView.trackIds.get(idxPair.get(i).dst);

                // If feature of the current view was NOT previously matched
                if (trkId == -1) {
                    // and corresponding feature of the match view was NOT previously matched
                    if (mtrkId == -1) {
                        // create NEW tracks
                        List<Integer> viewIds = new ArrayList<>();
                        List<Integer> kpIds = new ArrayList<>();
                        List<Boolean> inliers = new ArrayList<>();

                        viewIds.add(matchViewId);
                        kpIds.add(idxPair.get(i).dst);
                        inliers.add(true);
                        tracks.add(new Track(tracks.size(), 1, viewIds, kpIds, inliers));
                        mtrkId = tracks.size()-1;
                        matchView.trackIds.set(idxPair.get(i).dst, mtrkId);
                    }
                    // allow only one association per pair
                    if (!tracks.get(mtrkId).viewIds.contains(this.id)) {
                        // update existing track and add it to the current view
                        tracks.get(mtrkId).viewIds.add(this.id);
                        tracks.get(mtrkId).kpids.add(idxPair.get(i).src);
                        tracks.get(mtrkId).inliers.add(true);
                        tracks.get(mtrkId).length += 1;
                        this.trackIds.set(idxPair.get(i).src, mtrkId);
                    }
                }

                // If feature of the current view was previously matched
                else {
                    // and corresponding feature of the match view was NOT previously matched
                    if (mtrkId == -1) {
                        // allow only one association per pair
                        if (!tracks.get(trkId).viewIds.contains(matchViewId)) {
                            // update existing track and add it to the match view
                            tracks.get(trkId).viewIds.add(matchViewId);
                            tracks.get(trkId).kpids.add(idxPair.get(i).dst);
                            tracks.get(trkId).inliers.add(true);
                            tracks.get(trkId).length += 1;
                            matchView.trackIds.set(idxPair.get(i).dst, trkId);
                        }
                    }

                    // otherwise tracks are duplicates
                    else {
                        if (mtrkId != trkId) {

                            for (int j = 0; j < tracks.get(mtrkId).viewIds.size(); j++){
                                // allow only one association per pair
                                int oldId = tracks.get(mtrkId).viewIds.get(j);
                                if (!tracks.get(trkId).viewIds.contains(oldId)){
                                    // update existing track
                                    tracks.get(trkId).viewIds.add(oldId);
                                    tracks.get(trkId).kpids.add(tracks.get(mtrkId).kpids.get(j));
                                    tracks.get(trkId).inliers.add(tracks.get(mtrkId).inliers.get(j));
                                    tracks.get(trkId).length += 1;

                                }
                                // update track ids
                                views.get(oldId).trackIds.set(tracks.get(mtrkId).kpids.get(j), trkId);

                                tracks.get(mtrkId).valid = false;
                                tracks.get(mtrkId).viewIds = new ArrayList<>();
                                tracks.get(mtrkId).kpids = new ArrayList<>();
                                tracks.get(mtrkId).inliers = new ArrayList<>();
                                tracks.get(mtrkId).length = 0;
                            }

                            /*
                            for (int j = 0; j < tracks.get(trkId).viewIds.size(); j++){
                                // allow only one association per pair
                                int newId = tracks.get(trkId).viewIds.get(j);
                                if (!tracks.get(mtrkId).viewIds.contains(newId)){
                                    // update existing track
                                    tracks.get(mtrkId).viewIds.add(newId);
                                    tracks.get(mtrkId).kpids.add(tracks.get(trkId).kpids.get(j));
                                    tracks.get(mtrkId).inliers.add(tracks.get(trkId).inliers.get(j));
                                    tracks.get(mtrkId).length += 1;

                                }
                                // update track ids
                                views.get(newId).trackIds.set(tracks.get(trkId).kpids.get(j), mtrkId);

                                tracks.get(trkId).valid = false;
                                tracks.get(trkId).viewIds = new ArrayList<>();
                                tracks.get(trkId).kpids = new ArrayList<>();
                                tracks.get(trkId).inliers = new ArrayList<>();
                                tracks.get(trkId).length = 0;
                            }

                             */
                        }
                    }
                }
            }
        }
    }
    public List<AssociatedPair> get2Dmatches(View matchView, Connection conn){
        // collect list of matches
        List<AssociatedPair> matches = new ArrayList<>();
        for (int i = 0; i < conn.idxPair.size; i++) {
            int srcId = conn.idxPair.get(i).src;
            int dstId = conn.idxPair.get(i).dst;
            var p = new AssociatedPair(matchView.obs.get(dstId), this.obs.get(srcId));
            matches.add(p);
        }
        return matches;
    }
    public List<Point2D3D> get2D3Dmatches(List<Track> tracks, View matchView, Connection conn){
        // collect list of matches
        List<Point2D3D> matches = new ArrayList<>();
        for (int i = 0; i < conn.idxPair.size; i++) {
            int srcId = conn.idxPair.get(i).src;
            int dstId = conn.idxPair.get(i).dst;
            int trackId = matchView.trackIds.get(dstId);
            if (trackId != -1){
                if (tracks.get(trackId).valid) {
                    matches.add(new Point2D3D(this.obs.get(srcId), tracks.get(trackId).str));
                }
            }
        }
        return matches;
    }
    public int numOfTracks(List<Track> tracks){
        int cnt = 0;
        for (int trackId : this.trackIds) {
            if (trackId != -1) {
                if (tracks.get(trackId).valid) cnt += 1;
            }
        }
        return cnt;
    }
    public void triangulateTracks(List<Track> tracks, List<View> views, Config config){
        for (int trackId : this.trackIds) {
            if (trackId != -1) {
                tracks.get(trackId).triangulateN(views, config);
                tracks.get(trackId).filter(views, config);
            }
        }
    }
    public void filterTracks(List<Track> tracks, List<View> views, Config config){
        for (int trackId : this.trackIds) {
            if (trackId != -1) {
                tracks.get(trackId).filter(views, config);
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
        // todo estimate pose from best connection based on weight

        int matchViewId = this.conns.get(0).viewId;
        Se3_F64 motionAtoB = this.conns.get(0).getMotion();
        // concatenate pose
        Se3_F64 motionWorldToB =  views.get(matchViewId).pose.concat(motionAtoB, null);
        this.setPose(motionWorldToB);
    }

    public void viewTracks(List<Track> tracks, Config config) {
        Graphics2D vImg = this.img.createGraphics();
        for (int i = 0; i < this.kps.size(); i++) {
            if (this.trackIds.get(i) != -1) {
                if (tracks.get(this.trackIds.get(i)).valid) {
                    Point2D_F64 kprj = tracks.get(this.trackIds.get(i)).project(this, config);
                    VisualizeFeatures.drawPoint(vImg, this.kps.get(i).x, this.kps.get(i).y, 2, Color.BLUE, false);
                    VisualizeFeatures.drawPoint(vImg, kprj.x, kprj.y, 2, Color.RED, false);
                }
            }
        }
    }

    public static void viewViews(List<Track> tracks, List<View> views, Config config){
        for (View view : views){
            view.viewTracks(tracks, config);
            config.gui.addImage(view.img, view.file);
        }
        ShowImages.showWindow(config.gui,"detected features", true);
    }
}