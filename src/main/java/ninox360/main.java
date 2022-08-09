package ninox360;

import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.cloud.PointCloudReader;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.io.points.PointCloudIO;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.image.GrayF32;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.TwoAxisRgbPlane;
import boofcv.visualize.VisualizeData;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.FastAccess;
import org.ejml.data.DMatrixRMaj;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * saeed 2022-08-01: load and detect features in a directory
 *
 */
public class main {
    public static void main(String[] args) throws IOException {
        // Config
        String imageDirectory = "../dataset/03/";
        List<String> imageFiles = UtilIO.listImages( imageDirectory, true);
        List<View> views = new ArrayList<>();
        List<Track> tracks = new ArrayList<>();

        Config config = new Config(1000, 0.1, 0.5);
        config.getIntrinsic(UtilImageIO.loadImageNotNull(imageFiles.get(0)));

        SceneStructureMetric structure = new SceneStructureMetric(false);
        structure.cameras.grow();
        structure.setCamera(0, true, config.intrinsic);

        // Main Loop
        for (String imageFile : imageFiles){
            // add new view (detect features)
            int id = views.size();
            views.add(new View(id, imageFile, config));
            structure.views.grow();

            if (id != 0) {
                // create tracks
                int mid = id - 1;
                views.get(id).addConnection(mid, views.get(mid).dscs, config);
                views.get(id).mapTracks(tracks, views);

                // estimate pose
                views.get(id).estimatePose(structure, tracks, views, config);
            }

            // triangulate newly created tracks
            views.get(id).triangulation(tracks, views, config);
        }

        // Visualize
        for (View view : views){
            view.projectTracks(tracks, config);
            view.viewTracks();
            view.viewProjections();
            config.gui.addImage(view.img, view.file);
        }
        ShowImages.showWindow(config.gui,"detected features", true);

        // tracks
        int cnt = 0;
        for (Track track: tracks){
            if (track.valid) {
                config.viewer.addPoint(track.str.getX(), track.str.getY(), track.str.getZ(), 255);
                structure.points.grow();
                structure.setPoint(cnt, track.str.x, track.str.y, track.str.z);
                for (int camid:track.camids){
                    structure.connectPointToView(cnt, camid);
                }
                cnt += 1;
            }
        }

        SwingUtilities.invokeLater(() -> {
            BoofSwingUtil.visualizeCameras(structure, config.viewer);
            ShowImages.showWindow(config.viewer.getComponent(), "Refined Scene", true);
        });

        // Save point-cloud
        Track.saveCloud(tracks, config);
    }

}