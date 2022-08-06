package ninox360;

import boofcv.struct.geo.AssociatedPair;
import georegression.struct.point.Point3D_F64;
import java.util.List;

public class Track {
    int id;
    int length;
    List<Integer> camids;
    List<Integer> kpids;
    Point3D_F64 str = new Point3D_F64();
    boolean valid = false;

    public Track(int id, int length, List<Integer> camids, List<Integer> kpids){
        this.id = id;
        this.camids = camids;
        this.kpids = kpids;
        this.length = length;
    }

    // triangulates a track
    // initial version uses last two observations
    // assumes that only new tracks are being triangulated
    // to do: triangulated from multiple pairs and select the best
    // based on baseline, cheirality and geometric error
    public void triangulate(List<Camera> cameras, Config config){
        Point3D_F64 pt = new Point3D_F64();
        int mf = this.camids.get(this.length-2);
        int kf = this.camids.get(this.length-1);
        AssociatedPair match = new AssociatedPair(cameras.get(this.camids.get(0)).kps.get(this.kpids.get(0)),
                cameras.get(this.camids.get(1)).kps.get(this.kpids.get(1)));
        AssociatedPair matchNorm = new AssociatedPair();
        config.norm.compute(match.p1.x, match.p1.y, matchNorm.p1);
        config.norm.compute(match.p2.x, match.p2.y, matchNorm.p2);
        if (config.trian.triangulate(matchNorm.p1, matchNorm.p2, cameras.get(1).pose, pt)) {
            if (pt.z > 0) {
                this.str = pt;
                this.valid = true;
            }
        }
    }
}