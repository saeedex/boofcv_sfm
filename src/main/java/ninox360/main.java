package ninox360;

import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import javax.swing.*;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class main {
    public static void main(String[] args) throws IOException {
        // Config
        String imageDirectory = "../dataset/01/";
        List<String> imageFiles = UtilIO.listImages( imageDirectory, true);
        Config config = new Config(1000, 0.8, 2.0);
        if (!config.loadIntrinsic(imageDirectory)) config.getIntrinsic(UtilImageIO.loadImageNotNull(imageFiles.get(0)));

        // Detect and match features
        Recognizer recog = new Recognizer(imageDirectory, config);
        recog.detectFeat(imageFiles, config);
        recog.wrapFeat();
        recog.createModel();
        recog.createCraph();

        // Main Loop
        List<View> views = new ArrayList<>();
        List<Track> tracks = new ArrayList<>();

        for (String imageFile : imageFiles){
            int viewId = views.size();

            // Add new view
            views.add(new View(viewId, imageFile, config, recog.featList.get(viewId)));
            System.out.printf("Image[%2d]\n", viewId);

            if (viewId != 0) {
                // Add connections to the current view (matches, relative motion)
                views.get(viewId).addConnections(tracks, views, config, recog.conns.get(viewId));

                // Map existing tracks, create new tracks
                views.get(viewId).mapTracks(tracks, views);

                // estimate current view pose
                views.get(viewId).estimatePose(tracks, views, config);

                // Triangulate tracks visible in the current view
                views.get(viewId).triangulateTracks(tracks, views, config);

                // todo: pose graph optimization
                // Local bundle adjustment
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
        optimizer.graph.viewCloud(imageFiles);
        SwingUtilities.invokeLater(() -> {
            BoofSwingUtil.visualizeCameras(structure, optimizer.graph.getViewer());
            ShowImages.showWindow(optimizer.graph.getViewer().getComponent(), "Refined Scene", true);
        });

        // Save output
        optimizer.graph.saveCloud();
        //CalibrationIO.save(config.intrinsic, "intrinsic.yaml");
    }

}