package ninox360;

import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.WorldToCameraToPixel;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.DogArray_I32;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

/**
 * Track class: describes the relationship between 3D points and matched observations across all {@link View}.
 */
public class Track {
    /**
     * Track index
     */
    int id;
    /**
     * Track length
     */
    int length;
    /**
     * List of {@link View} indices where track is visible
     */
    DogArray_I32 viewIds;
    /**
     * List of observation indices in corresponding {@link View}
     */
    DogArray_I32 kpids;
    /**
     * Describes if the corresponding observation is an inlier (true:inlier)
     */
    List<Boolean> inliers;
    /**
     * Holds triangulated 3D point
     */
    Point3D_F64 str = new Point3D_F64();
    /**
     * Specifies if the track is valid. Valid tracks are triangulated and pass filter criteria (true:valid)
     */
    boolean valid = false;
    /**
     * Specifies if the track is a loop-closure (true:loop)
     */
    boolean loop = false;

    public Track(int id, int length, DogArray_I32 viewIds, DogArray_I32 kpids, List<Boolean> inliers) {
        this.id = id;
        this.viewIds = viewIds;
        this.kpids = kpids;
        this.length = length;
        this.inliers = inliers;
    }

    /**
     * Triangulates track
     * @param views List of {@link View}
     * @param config configuration
     */
    public void triangulateN(List<View> views, Config config) {
        var pt = new Point3D_F64();
        var matches = new ArrayList<Point2D_F64>();
        var poses = new ArrayList<Se3_F64>();
        for (int i = 0; i < this.viewIds.size(); i++) {
            View view = views.get(this.viewIds.get(i));
            matches.add(view.obs.get(this.kpids.get(i)));
            poses.add(view.worldToView);
        }

        // TODO note that you could triangulate this in homogenous coordinates
        if (config.trian.triangulate(matches, poses, pt)) {
            if (pt.z > 0) {
                this.str = pt;
                this.valid = true;
            }
        }
    }

    /**
     * Filters and invalidates track
     * @param views List of {@link View}
     * @param config configuration (camera intrinsic, threshold)
     */
    public void filter(List<View> views, Config config) {
        if (this.valid) {
            double eucDist;
            double res = 0;
            int totInliers = 0;
            for (int i = 0; i < this.viewIds.size(); i++) {
                View view = views.get(this.viewIds.get(i));
                Point2D_F64 prj = this.project(view, config);
                eucDist = prj.distance(view.kps.get(this.kpids.get(i)));
                if (eucDist < config.geoThreshold) {
                    totInliers += 1;
                    res += eucDist;
                }
                else this.inliers.set(i, false);
            }
            if(totInliers < 2) this.valid = false;
            if ((double)totInliers/this.length < 0.5) this.valid = false;
            if (res / totInliers > config.geoThreshold) this.valid = false;

        }
    }

    /**
     * Projects track to a {@link View}
     * @param view corresponding {@link View} to reproject to
     * @param config configuration (camera intrinsic)
     * @return projected pixel coordinates
     */
    public Point2D_F64 project(View view, Config config) {
        var kps = new Point2D_F64();
        WorldToCameraToPixel worldToPixel = PerspectiveOps.createWorldToPixel(config.intrinsic, view.worldToView);
        worldToPixel.transform(this.str, kps);
        return kps;
    }

    /**
     * Creates and adds new track to tracks list
     * @param tracks list of {@link Track}
     * @param view {@link View} of a camera where new track is observed
     * @param obsId matched feature {@link ImgFeats} index in view
     */
    public static void addTrack(List<Track> tracks, View view, int obsId){
        var viewIds = new DogArray_I32();
        var kpIds = new DogArray_I32();
        var inliers = new ArrayList<Boolean>();

        viewIds.add(view.id);
        kpIds.add(obsId);
        inliers.add(true);
        tracks.add(new Track(tracks.size(), 1, viewIds, kpIds, inliers));
        view.trackIds.set(obsId, tracks.size()-1);
    }

    /**
     * Adds a view to track
     * @param view {@link View} of a camera where new track is observed
     * @param obsId matched feature {@link ImgFeats} index in view
     */
    public void addView(View view, int obsId){
        // allow only one association per pair
        if (!this.viewIds.contains(view.id)) {
            // update existing track and add it to the current view
            this.viewIds.add(view.id);
            this.kpids.add(obsId);
            this.inliers.add(true);
            this.length += 1;
            view.trackIds.set(obsId, this.id);
        }
    }
    /**
     * Merges two tracks
     * @param views List of {@link View}
     * @param mtrack {@link Track} to be merged
     */
    public void merge(List<View> views, Track mtrack){
        for (int j = 0; j < mtrack.viewIds.size(); j++){
            // allow only one association per pair
            int newId = mtrack.viewIds.get(j);
            if (!this.viewIds.contains(newId)){
                // update existing track
                this.viewIds.add(newId);
                this.kpids.add(mtrack.kpids.get(j));
                this.inliers.add(mtrack.inliers.get(j));
                this.length += 1;
                // update track ids
                views.get(newId).trackIds.set(mtrack.kpids.get(j), this.id);
                mtrack.valid = false;
            }
        }
    }
}