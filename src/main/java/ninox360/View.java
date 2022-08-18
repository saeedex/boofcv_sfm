package ninox360;

import boofcv.gui.BoofSwingUtil;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.gui.image.ImageZoomPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.geo.Point2D3D;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import lombok.Getter;
import lombok.Setter;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.DogArray_I32;
import org.ddogleg.struct.FastAccess;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Connection class: specifies the relationship between to views.
 * Holds their matched observations and relative motion.
 */
final class Connection {
    /**
     * Connected {@link View} index
     */
    int viewId;
    /**
     * List of matched observation indices
     */
    FastAccess<AssociatedIndex> idxPair;
    /**
     * List of inlier matched observation indices
     */
    List<AssociatedIndex> inlierPair;
    /**
     * Relative motion from connected {@link View} to parent
     */
    @Getter @Setter Se3_F64 motion;
    /**
     * Weight of the connection [0-1]
     */
    @Getter @Setter private double weight;

    public Connection(int viewId, FastAccess<AssociatedIndex> idxPair) {
        this.viewId = viewId;
        this.idxPair = idxPair;
    }

    /**
     * estimates the relative motion of connection (from match to parent).
     * saves the match inlier set
     *
     * @param viewId parent {@link View} index
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     * @return the weight of the connection (based on geometric inliers)
     */
    public double estimateMotion(int viewId, List<Track> tracks, List<View> views, Config config) {
        View matchView = views.get(this.viewId);
        View view = views.get(viewId);
        Se3_F64 motionAtoB;
        Se3_F64 motionWorldToB = new Se3_F64();
        double weight;

        // if no valid tracks available initialize pose using epimotion
        if (matchView.numOfTracks(tracks) == 0) {
            List<AssociatedPair> matches2D = view.get2Dmatches(matchView, this);
            List<AssociatedPair> inliers = new ArrayList<>();

            // Estimate motion up to a scale invariance using an Essential matrix
            config.epiMotion.setIntrinsic(0, config.intrinsic);
            config.epiMotion.setIntrinsic(1, config.intrinsic);

            // estimate relative camera motion
            if (!config.epiMotion.process(matches2D))
                throw new RuntimeException("Motion estimation failed");
            motionAtoB = config.epiMotion.getModelParameters();

            // set match inliers and updated weight
            inliers.addAll(config.epiMotion.getMatchSet());
            weight = (double)inliers.size() / (double)matches2D.size();
            this.inlierPair = new ArrayList<>();
            for (int i = 0; i < inliers.size(); i++) {
                this.inlierPair.add(this.idxPair.get(config.epiMotion.getInputIndex(i)));
            }
        }
        // else use pnp to estimate pose
        else {
            List<Point2D3D> matches2D3D = view.get2D3Dmatches(tracks, matchView, this);
            List<Point2D3D> inliers = new ArrayList<>();

            // estimate camera motion
            config.estimatePnP.setIntrinsic(0, config.intrinsic);
            if (matches2D3D.size() < config.estimatePnP.getMinimumSize())
                System.out.println(config.estimatePnP.getMatchSet().size());
            if (!config.estimatePnP.process(matches2D3D))
                throw new RuntimeException("Motion estimation failed");

            // refine the motion estimate using non-linear optimization
            if (!config.refinePnP.fitModel(config.estimatePnP.getMatchSet(), config.estimatePnP.getModelParameters(), motionWorldToB))
                throw new RuntimeException("Refine failed!?!?");

            // compute relative motion of current camera for the connection
            Se3_F64 motionBtoWorld = motionWorldToB.invert(null);
            Se3_F64 motionWorldToA = matchView.motionWorldToView;
            Se3_F64 motionBtoA = motionBtoWorld.concat(motionWorldToA, null);
            motionAtoB = motionBtoA.invert(null);

            // set match inliers and updated weight
            inliers.addAll(config.estimatePnP.getMatchSet());
            weight = (double)inliers.size() / (double)matches2D3D.size();
            List<AssociatedIndex> newPair = new ArrayList<>();
            for (int i = 0; i < inliers.size(); i++) {
                newPair.add(this.inlierPair.get(config.estimatePnP.getInputIndex(i)));
            }
            this.inlierPair = newPair;
        }

        // set relative motion
        this.setMotion(motionAtoB);
        return weight;
    }
}

/**
 * View class: specifies a view from a camera at a particular point.
 * Holds captured image, observed features, camera pose, and connections with other views.
 */
public class View {
    /**
     * View index
     */
    int id;
    /**
     * Associated image file name
     */
    String file;
    /**
     * Associated image
     */
    BufferedImage img;
    /**
     * {@link ImgFeats} Key points pixel coordinates
     */
    List<Point2D_F64> kps;
    /**
     * Normalized key points pixel coordinates
     */
    List<Point2D_F64> obs = new ArrayList<>();
    /**
     * {@link ImgFeats} Descriptors of key points
     */
    DogArray<TupleDesc_F64> dscs;
    /**
     * Which track this feature is an observation of. -1 means unassigned
     */
    DogArray_I32 trackIds;
    /**
     * View pose
     */
    @Getter Se3_F64 motionWorldToView = new Se3_F64();
    /**
     * List of {@link Connection} (with other views) of the view
     */
    List<Connection> conns = new ArrayList<>();

    public View(int id, String file, Config config, ImgFeats feat) {
        BufferedImage img = UtilImageIO.loadImageNotNull(file);
        this.id = id;
        this.file = file;
        this.img = img;
        this.kps = feat.kps();
        this.dscs = feat.dscs();
        this.trackIds = feat.trackIds();
        this.normKps(config);
    }

    /**
     * Adds a {@link Connection} to view
     * by matching features of candidate connections
     * and checking the ratio of matched features.
     * Connections passing the criteria are added
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     * @param conviews list of candidate {@link View} indices
     */
    public void addConnections(List<Track> tracks, List<View> views, Config config, List<Integer> conviews) {
        double numFeats = this.dscs.size();
        for (Integer matchViewId : conviews) {
            FastAccess<AssociatedIndex> idxPair = Features.match(this.dscs, views.get(matchViewId).dscs, config);
            double weight = idxPair.size / numFeats;

            // add previous view connection by default
            if (this.conns.size() == 0) {
                Connection conn = new Connection(matchViewId, idxPair);
                conn.setWeight(conn.estimateMotion(this.id, tracks, views, config));
                this.conns.add(conn);
            }

            // add loop closure connection if selection criteria is passed:
            // checking match ratio (above 30% is acceptable)
            // checking geometric inliers (above 50% is acceptable)
            else if (weight > 0.2) {
                Connection conn = new Connection(matchViewId, idxPair);
                conn.setWeight(conn.estimateMotion(this.id, tracks, views, config));
                if (conn.getWeight() > 0.3) this.conns.add(conn);
            }
        }
    }

    public Connection selectBestConnection() {
        Connection bestConn = conns.get(0);
        if (conns.size() > 1) {
            for (int i = 1; i < conns.size(); i++) {
                if (conns.get(i).getWeight() > bestConn.getWeight()) {
                    bestConn = conns.get(i);
                }
            }
        }
        return bestConn;
    }

    /**
     * Maps a 2D match {@link ImgFeats} with associated {@link Track}.
     * Creates new {@link Track} if no track is accociated.
     * Only inlier matches are used.
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     */
    public void mapTracks(List<Track> tracks, List<View> views) {
        for (Connection conn : this.conns) {
            int matchViewId = conn.viewId;
            System.out.printf("  %6s: weight=%.2f assoc=%d inliers=%d\n",
                    matchViewId, conn.getWeight(), conn.idxPair.size, conn.inlierPair.size());
            View matchView = views.get(matchViewId);

            // We can use inlier set here. Works well for all datasets except for 05.
            // The matches are still being filtered after triangulation. Only inliers are added to SBA.
            FastAccess<AssociatedIndex> idxPair = conn.idxPair;
            //List<AssociatedIndex> idxPair = conn.inlierPair;

            for (int i = 0; i < idxPair.size(); i++) {
                int trkId = this.trackIds.get(idxPair.get(i).src);
                int mtrkId = matchView.trackIds.get(idxPair.get(i).dst);

                // If feature of the current view was NOT previously matched
                if (trkId == -1) {
                    // and corresponding feature of the match view was NOT previously matched
                    if (mtrkId == -1) {
                        // create new track
                        Track.addTrack(tracks, matchView, idxPair.get(i).dst);
                        mtrkId = tracks.size() - 1;
                    }
                    // add current view to track
                    tracks.get(mtrkId).addView(this, idxPair.get(i).src);
                }
                // If feature of the current view was previously matched
                else {
                    // and corresponding feature of the match view was NOT previously matched
                    if (mtrkId == -1) {
                        // add match view to track
                        tracks.get(trkId).addView(matchView, idxPair.get(i).dst);
                    }
                    // otherwise tracks are revisited
                    else {
                        tracks.get(mtrkId).loop = true;
                        tracks.get(trkId).loop = true;
                        if (mtrkId != trkId) {
                            // merge duplicates
                            if (tracks.get(mtrkId).valid)
                                tracks.get(mtrkId).merge(views, tracks.get(trkId));
                            else
                                tracks.get(trkId).merge(views, tracks.get(mtrkId));
                        }
                    }
                }
            }
        }
    }

    /**
     * Collects the list of 2D matches {@link ImgFeats} (observation-observation)
     * between view and matchView
     *
     * @param matchView matched {@link View}
     * @param conn associated {@link Connection}
     * @return list of 2D matches
     */
    public List<AssociatedPair> get2Dmatches(View matchView, Connection conn) {
        // collect list of matches
        var matches = new ArrayList<AssociatedPair>();
        for (int i = 0; i < conn.idxPair.size; i++) {
            int srcId = conn.idxPair.get(i).src;
            int dstId = conn.idxPair.get(i).dst;
            var p = new AssociatedPair(matchView.obs.get(dstId), this.obs.get(srcId));
            matches.add(p);
        }
        return matches;
    }

    /**
     * Collects the list of 2D {@link ImgFeats} to 3D matches {@link Track} (observation-track)
     * between view and matchView
     *
     * @param tracks list of {@link Track}
     * @param matchView matched {@link View}
     * @param conn associated {@link Connection}
     * @return list of 2D-3D matches
     */
    public List<Point2D3D> get2D3Dmatches(List<Track> tracks, View matchView, Connection conn) {
        // collect list of matches
        var matches = new ArrayList<Point2D3D>();
        conn.inlierPair = new ArrayList<>();
        for (int i = 0; i < conn.idxPair.size; i++) {
            int srcId = conn.idxPair.get(i).src;
            int dstId = conn.idxPair.get(i).dst;
            int trackId = matchView.trackIds.get(dstId);
            if (trackId != -1) {
                if (tracks.get(trackId).valid) {
                    matches.add(new Point2D3D(this.obs.get(srcId), tracks.get(trackId).str));
                    // temporarily save the index pairs associated with valid tracks
                    conn.inlierPair.add(conn.idxPair.get(i));
                }
            }
        }
        return matches;
    }

    /**
     * Counts valid {@link Track} visible in {@link View}
     *
     * @param tracks list of {@link Track}
     * @return integer count of valid {@link Track}
     */
    public int numOfTracks(List<Track> tracks) {
        return trackIds.count(trackId -> trackId != -1 && tracks.get(trackId).valid);
    }

    /**
     * triangulates tracks visible in view
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public void triangulateTracks(List<Track> tracks, List<View> views, Config config) {
        trackIds.forEach(trackId -> {
            if (trackId != -1) {
                tracks.get(trackId).triangulateN(views, config);
                tracks.get(trackId).filter(views, config);
            }
        });
    }

    /**
     * filters and invalidates tracks visible in view
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public void filterTracks(List<Track> tracks, List<View> views, Config config) {
        trackIds.forEach(trackId -> {
            if (trackId != -1) {
                tracks.get(trackId).filter(views, config);
            }
        });
    }

    /**
     * Converts {@link ImgFeats} kps to normalized image coordinate
     *
     * @param config configuration (camera intrinsic)
     */
    public void normKps(Config config) {
        var pointNorm = new Point2D_F64();
        for (Point2D_F64 kp : this.kps) {
            config.norm.compute(kp.x, kp.y, pointNorm);
            this.obs.add(pointNorm.copy());
        }
    }

    /**
     * Estimate {@link View} pose by concatenation.
     * Selects best {@link Connection} based on weight.
     *
     * @param views list of {@link View}
     */
    public void estimatePose(List<View> views) {
        Connection bestConn = selectBestConnection();
        int matchViewId = bestConn.viewId;
        Se3_F64 motionAtoB = bestConn.getMotion();

        // concatenate pose
        Se3_F64 motionWorldToB = views.get(matchViewId).motionWorldToView.concat(motionAtoB, null);
        this.motionWorldToView.setTo(motionWorldToB);
    }

    /**
     * Visualizes {@link Track} visible in {@link View} showing differences between predicted and observed locations
     *
     * @param tracks list of {@link Track}
     * @param config configuration
     */
    public void viewTracks(List<Track> tracks, Config config) {
        Graphics2D g2 = this.img.createGraphics();
        BoofSwingUtil.antialiasing(g2);
        g2.setStroke(new BasicStroke(2));
        int r = 4; // radius of point
        var line = new Line2D.Double();

        for (int i = 0; i < this.kps.size(); i++) {
            if (this.trackIds.get(i) != -1) {
                Track track = tracks.get(this.trackIds.get(i));
                if (!track.valid || !track.viewIds.contains(this.id) || !track.inliers.get(track.viewIds.indexOf(this.id)))
                    continue;

                // reproject track
                Point2D_F64 kprj = track.project(this, config);
                // Draw a line indicating reprojection error
                g2.setColor(Color.BLUE);
                line.setLine(kps.get(i).x, kps.get(i).y, kprj.x, kprj.y);
                g2.draw(line);
                VisualizeFeatures.drawPoint(g2, this.kps.get(i).x, this.kps.get(i).y, r, Color.BLUE, false);
                VisualizeFeatures.drawPoint(g2, kprj.x, kprj.y, r, Color.RED, false);
                if (track.loop) VisualizeFeatures.drawPoint(g2, kprj.x, kprj.y, r, Color.GREEN, false);
            }
        }
    }

    /**
     * Visualizes all {@link View} with observations
     *
     * @param tracks list of {@link Track}
     * @param views list of {@link View}
     * @param config configuration
     */
    public static void viewViews(List<Track> tracks, List<View> views, Config config) {
        for (View view : views) {
            view.viewTracks(tracks, config);

            // Display the image and let you use a mouse scroll wheel to zoom in and out
            var imagePanel = new ImageZoomPanel();
            imagePanel.setImage(view.img);
            imagePanel.addMouseWheelListener(e -> imagePanel.setScale(BoofSwingUtil.mouseWheelImageZoom(imagePanel.getScale(), e)));
            config.gui.addItem(imagePanel, new File(view.file).getName());
        }
        ShowImages.showWindow(config.gui, "detected features", true);
    }
}