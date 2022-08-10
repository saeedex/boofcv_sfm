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

        Config config = new Config(500, 0.1, 0.8);
        config.getIntrinsic(UtilImageIO.loadImageNotNull(imageFiles.get(0)));

        // Main Loop
        for (String imageFile : imageFiles){
            // add new view (detect features)
            int viewId = views.size();
            views.add(new View(viewId, imageFile, config));

            if (viewId != 0) {
                // create tracks
                int mid = viewId - 1;
                views.get(viewId).addConnection(mid, views.get(mid).dscs, config);
                views.get(viewId).mapTracks(tracks, views);

                // estimate pose
                views.get(viewId).estimatePose(tracks, views, config);
            }

            // triangulate newly created tracks
            views.get(viewId).triangulateTracks(tracks, views, config);
        }

        // Initialize structure and observations for Bundle adjustment
        SceneStructureMetric structure = new SceneStructureMetric(false);
        SceneObservations observations = new SceneObservations();
        View.wrapScene(structure, observations, tracks, views, config);

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