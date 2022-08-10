package ninox360;

import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.cloud.PointCloudReader;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.io.points.PointCloudIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.image.GrayF32;
import boofcv.visualize.PointCloudViewer;
import boofcv.visualize.TwoAxisRgbPlane;
import boofcv.visualize.VisualizeData;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.optimization.lm.ConfigLevenbergMarquardt;
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
        SceneObservations observations = new SceneObservations();

        //structure.cameras.grow();
        //structure.setCamera(0, false, config.intrinsic);

        // Main Loop
        for (String imageFile : imageFiles){
            // add new view (detect features)
            int viewId = views.size();
            views.add(new View(viewId, imageFile, config));
            //structure.views.grow();
            //observations.views.grow();

            if (viewId != 0) {
                // create tracks
                int mid = viewId - 1;
                views.get(viewId).addConnection(mid, views.get(mid).dscs, config);
                views.get(viewId).mapTracks(structure, observations, tracks, views);

                // estimate pose
                views.get(viewId).estimatePose(structure, tracks, views, config);
            }

            // triangulate newly created tracks
            views.get(viewId).triangulateTracks(structure, observations, tracks, views, config);
        }

        // Initialize
        int cnt = 0;
        for (Track track: tracks)
            if (track.valid) cnt +=1;
        structure.initialize(1, views.size(), cnt);
        observations.initialize(views.size());

        // set cameras
        structure.setCamera(0, false, config.intrinsic);

        // set views
        structure.setView(0, 0, true, views.get(0).pose);
        for (int viewId = 1; viewId < views.size(); viewId++) {
            structure.setView(viewId, 0, true, views.get(viewId).conns.get(0).getMotion(),
                    views.get(viewId).conns.get(0).viewId);
        }
        // set points
        for (Track track: tracks){
            if (track.valid){
                structure.points.grow();
                track.setValidId(structure.points.size-1);
                structure.setPoint(track.validId, track.str.x, track.str.y, track.str.z);

                for (int i = 0; i < track.viewIds.size(); i++) {
                    int viewId = track.viewIds.get(i);
                    observations.views.get(viewId).add(track.validId,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).x,
                            (float)views.get(viewId).kps.get(track.kpids.get(i)).y);
                    structure.connectPointToView(track.validId, viewId);
                }
            }
        }

        // Bundle adjustment
        View.bundleAdjustment(structure, observations, config);

        // Visualize
        View.viewViews(tracks, views, config);
        Track.addCloud2viewer(structure, config);
        SwingUtilities.invokeLater(() -> {
            BoofSwingUtil.visualizeCameras(structure, config.viewer);
            ShowImages.showWindow(config.viewer.getComponent(), "Refined Scene", true);
        });

        // Save point-cloud
        Track.saveCloud(structure, config);

    }

}