package ninox360;

import boofcv.alg.cloud.PointCloudReader;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.WorldToCameraToPixel;
import boofcv.io.points.PointCloudIO;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.struct.geo.AssociatedPair;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

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

    /**
     * triangulates a track. Initial version uses last two observations assumes
     * that only new tracks are being triangulated
     * based on baseline, cheirality and geometric error.
     */
    /*
    public void triangulate(List<Camera> cameras, Config config){
        Point3D_F64 mcampt = new Point3D_F64();
        Point3D_F64 pt = new Point3D_F64();
        int mid = this.length - 2;
        int id = this.length - 1;
        int mcamid = this.camids.get(mid);
        int camid = this.camids.get(id);
        AssociatedPair match = new AssociatedPair(cameras.get(mcamid).obs.get(this.kpids.get(mid)),
                cameras.get(camid).obs.get(this.kpids.get(id)));

        Se3_F64 motionBtoWorld = cameras.get(mcamid).pose.invert(null);
        if (config.trian.triangulate(match.p1, match.p2, cameras.get(camid).conns.get(0).getmotion(), mcampt)) {
            if (mcampt.z > 0) {
                SePointOps_F64.transform(motionBtoWorld, mcampt, pt);
                this.str = mcampt;
                this.valid = true;
            }
        }
    }

     */
    public void triangulateN(List<View> views, Config config){
        Point3D_F64 pt = new Point3D_F64();
        List<Point2D_F64> matches = new ArrayList<>();
        List<Se3_F64> poses = new ArrayList<>();
        for (int i = 0; i < this.camids.size(); i++) {
            View view = views.get(this.camids.get(i));
            matches.add(view.obs.get(this.kpids.get(i)));
            poses.add(view.pose);
        }

        if (config.trian.triangulate(matches, poses, pt)) {
            if (pt.z > 0) {
                this.str = pt;
                this.valid = true;

                double diffx;
                double diffy;
                double res = 0;
                for (int i = 0; i < this.camids.size(); i++) {
                    View view = views.get(this.camids.get(i));
                    Point2D_F64 prj = this.project(view, config);

                    diffx = (prj.x - view.kps.get(this.kpids.get(i)).x);
                    diffy = (prj.y - view.kps.get(this.kpids.get(i)).y);
                    res += sqrt(diffx*diffx + diffy*diffy);
                }
                res = res/this.camids.size();
                if(res > 1.0) this.valid = false;
            }
        }
    }
    public Point2D_F64 project(View view, Config config){
        Point2D_F64 kps = new Point2D_F64();
        WorldToCameraToPixel worldToPixel = PerspectiveOps.createWorldToPixel(config.intrinsic, view.pose);
        worldToPixel.transform(this.str, kps);
        return kps;
    }

    public static void saveCloud(List<Track> tracks, Config config) throws IOException {
        List<Point3dRgbI_F64> cloud = new ArrayList<>();
        for (Track track: tracks){
            if (track.valid) {
                cloud.add(new Point3dRgbI_F64(track.str.getX(), track.str.getY(), track.str.getZ(), 255));
            }
        }
        cloud.toArray(new Point3dRgbI_F64[0]);
        OutputStream out = new FileOutputStream("saved_cloud.ply");
        PointCloudIO.save3D(PointCloudIO.Format.PLY, PointCloudReader.wrapF64RGB(cloud), true, out);
    }
}