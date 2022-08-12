package ninox360;

import boofcv.abst.geo.bundle.BundleAdjustment;
import boofcv.abst.geo.bundle.ScaleSceneStructure;
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.cloud.PointCloudReader;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.bundle.BundleAdjustmentOps;
import boofcv.alg.geo.bundle.cameras.BundlePinholeBrown;
import boofcv.factory.geo.ConfigBundleAdjustment;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.calibration.CalibrationIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.io.points.PointCloudIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.Point3dRgbI_F64;
import boofcv.struct.calib.CameraPinholeBrown;
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
        String imageDirectory = "../dataset/01/";
        List<String> imageFiles = UtilIO.listImages( imageDirectory, true);
        List<View> views = new ArrayList<>();
        List<Track> tracks = new ArrayList<>();


        Config config = new Config(1000, 0.8, 2.0);
        if (!config.loadIntrinsic(imageDirectory)) config.getIntrinsic(UtilImageIO.loadImageNotNull(imageFiles.get(0)));
        // Main Loop
        for (String imageFile : imageFiles){

            // add new view (detect features)
            int viewId = views.size();
            //System.out.println(viewId);
            views.add(new View(viewId, imageFile, config));

            if (viewId != 0) {
                // map existing tracks
                // create new tracks
                int mid = viewId - 1;
                views.get(viewId).addConnection(mid, views.get(mid).dscs, config);
                if (viewId == 89) views.get(viewId).addConnection(0, views.get(0).dscs, config); //loop closure here

                views.get(viewId).mapTracks(tracks, views);

                // estimate current view pose
                views.get(viewId).estimatePose(tracks, views, config);

                // triangulate tracks visible in current view
                views.get(viewId).triangulateTracks(tracks, views, config);

                // local bundle adjustment
                Optimizer optimizer = new Optimizer(true);
                optimizer.initGraph(tracks, views);
                optimizer.wrapGraph(tracks, views, config);
                optimizer.process();
                optimizer.unwrapGraph(tracks, views, config);
            }
            System.out.printf("Registered view: %d\n", viewId);
        }
        // Global bundle adjustment
        Optimizer optimizer = new Optimizer(false);
        optimizer.initGraph(tracks, views);
        optimizer.wrapGraph(tracks, views, config);
        optimizer.process();
        optimizer.unwrapGraph(tracks, views, config);

        // Visualize
        SceneStructureMetric structure = optimizer.graph.getStructure();
        View.viewViews(tracks, views, config);
        Track.addCloud2viewer(structure, config);
        SwingUtilities.invokeLater(() -> {
            BoofSwingUtil.visualizeCameras(structure, config.viewer);
            ShowImages.showWindow(config.viewer.getComponent(), "Refined Scene", true);
        });

        // Save output
        Track.saveCloud(structure, config);
        //CalibrationIO.save(config.intrinsic, "intrinsic.yaml");
    }

}