package ninox360;

import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.alg.cloud.PointCloudReader;
import boofcv.factory.geo.ConfigTriangulation;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.image.ShowImages;
import boofcv.io.points.PointCloudIO;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.TwoAxisRgbPlane;
import boofcv.visualize.VisualizeData;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point3D_F64;
import org.ddogleg.struct.DogArray;

import java.awt.*;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
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

    public static void triangulate(List<Track> tracks, List<Camera> cameras, Config config) throws IOException {
        List<AssociatedPair> matches = new ArrayList<>();
        for (Track track : tracks) {
            var p = new AssociatedPair(cameras.get(track.camids.get(0)).kps.get(track.kpids.get(0)),
                    cameras.get(track.camids.get(1)).kps.get(track.kpids.get(1)));
            matches.add(p);
        }
        List<AssociatedPair> matchesNorm = pose.convertToNormalizedCoordinates(matches, config.intrinsic);

        List<Point3D_F64> points = new ArrayList<>();
        List<Integer> colors = new ArrayList<>();

        for (AssociatedPair p : matchesNorm) {
            Point3D_F64 pt = new Point3D_F64();
            if (!config.trian.triangulate(p.p1, p.p2, cameras.get(1).pose, pt))
                continue;
            if (pt.z > 0) {
                points.add(pt);
                colors.add(255);
            }
        }

        // for test
        System.out.println(points.size());
        PointCloudViewer viewer = VisualizeData.createPointCloudViewer();
        viewer.setFog(true);
        viewer.setColorizer(new TwoAxisRgbPlane.Z_XY(1.0).fperiod(40));
        viewer.setDotSize(1);
        //viewer.setTranslationStep(0.15);
        viewer.addCloud(( idx, p ) -> p.setTo(points.get(idx)), colors::get, colors.size());
        viewer.setCameraHFov(UtilAngle.radian(60));
        viewer.getComponent().setPreferredSize(new Dimension(600, 600));
        ShowImages.showWindow(viewer.getComponent(), "Refined Scene", true);
        var copy = new DogArray<>(Point3dRgbI_F64::new);
        viewer.copyCloud(copy);
        OutputStream out = new FileOutputStream("saved_cloud.ply");
        PointCloudIO.save3D(PointCloudIO.Format.PLY, PointCloudReader.wrapF64RGB(copy.toList()), true, out);
    }
}