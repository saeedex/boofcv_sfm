package ninox360;

import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.WorldToCameraToPixel;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.DogArray_I32;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

// TODO describe what this is
public class Track {
    int id;
    int length;
    DogArray_I32 viewIds;
    DogArray_I32 kpids;
    List<Boolean> inliers;
    Point3D_F64 str = new Point3D_F64();
    boolean valid = false;

    public Track(int id, int length, DogArray_I32 viewIds, DogArray_I32 kpids, List<Boolean> inliers) {
        this.id = id;
        this.viewIds = viewIds;
        this.kpids = kpids;
        this.length = length;
        this.inliers = inliers;
    }

    /**
     * triangulates a track.
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

    public void filter(List<View> views, Config config) {
        if (this.valid) {
            double eucDist;
            double res = 0;
            int totInliers = 0;
            for (int i = 0; i < this.viewIds.size(); i++) {
                View view = views.get(this.viewIds.get(i));
                Point2D_F64 prj = this.project(view, config);
                eucDist = prj.distance(view.kps.get(this.kpids.get(i)));
                if (eucDist > config.geoThreshold) {
                    this.inliers.set(i, false);
                    totInliers += 1;
                    res += eucDist;
                }
            }
            //if(totInliers < 2) this.valid = false;
            if (res / totInliers > config.geoThreshold) this.valid = false;
        }
    }

    public Point2D_F64 project(View view, Config config) {
        var kps = new Point2D_F64();
        WorldToCameraToPixel worldToPixel = PerspectiveOps.createWorldToPixel(config.intrinsic, view.worldToView);
        worldToPixel.transform(this.str, kps);
        return kps;
    }
}