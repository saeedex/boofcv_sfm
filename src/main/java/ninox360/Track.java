package ninox360;

import boofcv.abst.geo.bundle.SceneStructureCommon;
import boofcv.abst.geo.bundle.SceneStructureMetric;
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
    List<Integer> viewIds;
    List<Integer> kpids;
    List<Boolean> inliers;

    Point3D_F64 str = new Point3D_F64();
    boolean valid = false;

    public Track(int id, int length, List<Integer> viewIds, List<Integer> kpids, List<Boolean> inliers){
        this.id = id;
        this.viewIds = viewIds;
        this.kpids = kpids;
        this.length = length;
        this.inliers = inliers;
    }

    /**
     * triangulates a track.
     */
    public void triangulateN(List<View> views, Config config){
        Point3D_F64 pt = new Point3D_F64();
        List<Point2D_F64> matches = new ArrayList<>();
        List<Se3_F64> poses = new ArrayList<>();
        for (int i = 0; i < this.viewIds.size(); i++) {
            View view = views.get(this.viewIds.get(i));
            matches.add(view.obs.get(this.kpids.get(i)));
            poses.add(view.pose);
        }

        if (config.trian.triangulate(matches, poses, pt)) {
            if (pt.z > 0) {
                this.str = pt;
                this.valid = true;
            }
        }
    }
    public void filter(List<View> views, Config config){
        if (this.valid){
            double eucDist;
            double res = 0;
            int totInliers = 0;
            for (int i = 0; i < this.viewIds.size(); i++) {
                View view = views.get(this.viewIds.get(i));
                Point2D_F64 prj = this.project(view, config);
                eucDist = prj.distance(view.kps.get(this.kpids.get(i)));
                if (eucDist > config.geoThreshold) {
                    this.inliers.set(i, false);
                    totInliers +=1;
                    res += eucDist;
                }
            }
            //if(totInliers < 2) this.valid = false;
            if(res/totInliers > config.geoThreshold) this.valid = false;
        }
    }
    public Point2D_F64 project(View view, Config config){
        Point2D_F64 kps = new Point2D_F64();
        WorldToCameraToPixel worldToPixel = PerspectiveOps.createWorldToPixel(config.intrinsic, view.pose);
        worldToPixel.transform(this.str, kps);
        return kps;
    }

    public static void saveCloud(SceneStructureMetric structure, Config config) throws IOException {
        List<Point3dRgbI_F64> cloud = new ArrayList<>();
        for (int i = 0; i < structure.points.size; i++) {
            Point3D_F64 world = new Point3D_F64();
            structure.points.get(i).get(world);
            cloud.add(new Point3dRgbI_F64(world.getX(), world.getY(), world.getZ(), 255));
        }
        cloud.toArray(new Point3dRgbI_F64[0]);
        OutputStream out = new FileOutputStream("saved_cloud.ply");
        PointCloudIO.save3D(PointCloudIO.Format.PLY, PointCloudReader.wrapF64RGB(cloud), true, out);
    }

    public static void addCloud2viewer(SceneStructureMetric structure, Config config){
        for (int i = 0; i < structure.points.size; i++) {
            Point3D_F64 world = new Point3D_F64();
            structure.points.get(i).get(world);
            config.viewer.addPoint(world.getX(), world.getY(), world.getZ(), 255);
        }
    }
}