package ninox360;

import boofcv.gui.feature.VisualizeFeatures;
import boofcv.struct.feature.TupleDesc_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.DogArray;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.List;

public class Camera {
    int id;
    String file;
    BufferedImage img;
    List<Point2D_F64> kps;
    DogArray<TupleDesc_F64> dscs;
    List<Integer> trackids;
    Se3_F64 pose;

    public Camera(int id, String file, BufferedImage img, List<Point2D_F64> kps, DogArray<TupleDesc_F64> dscs, List<Integer> trackids){
        this.id = id;
        this.file = file;
        this.img = img;
        this.kps = kps;
        this.dscs = dscs;
        this.trackids = trackids;
    }

    public void viewkps() {
        Graphics2D vimg = this.img.createGraphics();
        for (int i = 0; i < this.kps.size(); i++) {
            VisualizeFeatures.drawPoint(vimg, this.kps.get(i).x, this.kps.get(i).y, 2, Color.RED, false);
        }
    }

    public void viewtracks() {
        Graphics2D vimg = this.img.createGraphics();
        for (int i = 0; i < this.kps.size(); i++) {
            if (this.trackids.get(i) != -1) {
                VisualizeFeatures.drawPoint(vimg, this.kps.get(i).x, this.kps.get(i).y, 2, Color.BLUE, false);
            }
        }
    }

    public void setpose(Se3_F64 pose){
        this.pose = pose;
    }
}