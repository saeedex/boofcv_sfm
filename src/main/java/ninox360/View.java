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
    Se3_F64 motion;
    double weight;

    public Connection(int viewId, FastAccess<AssociatedIndex> idxPair, double weight) {
        this.viewId = viewId;
        this.idxPair = idxPair;
        this.weight = weight;
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

    public void addConnection(List<View> views, Config config, List<Integer> conviews){
        for (Integer matchViewId: conviews){
            FastAccess<AssociatedIndex> idxPair = features.match(this.dscs, views.get(matchViewId).dscs, config);
            double matchFraction = idxPair.size/(double)this.dscs.size();
            if (this.conns.size() == 0)

                this.conns.add(new Connection(matchViewId, idxPair, matchFraction));
            else if (matchFraction > 0.4) {
                this.conns.add(new Connection(matchViewId, idxPair, matchFraction));
            }
        }

    }
    public void mapTracks(List<Track> tracks, List<View> views) {
        //System.out.println(this.id);
        for (Connection conn : this.conns) {
            int matchViewId = conn.viewId;
            System.out.printf("  %4s matches=%.2f\n", matchViewId, conn.weight);

            View matchView = views.get(matchViewId);
            FastAccess<AssociatedIndex> idxPair = conn.idxPair;

            for (int i = 0; i < idxPair.size; i++) {
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
        int mid = this.conns.get(0).viewId;
        View matchView = views.get(mid);
        Se3_F64 motionAtoB;
        Se3_F64 motionWorldToB = new Se3_F64();

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