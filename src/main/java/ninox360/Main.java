package ninox360;

import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import javax.swing.*;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Main class which opens the data and puts everything together.
 */
public class Main {

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            // Let the user select an image using a GUI or use the one provided as an argument
            File imageDirectory;
            if (args.length == 0) {
                imageDirectory = BoofSwingUtil.openFileChooser("SFM", BoofSwingUtil.FileTypes.DIRECTORIES);
                if (imageDirectory == null)
                    return;
            } else {
                imageDirectory = new File(args[0]);
            }

            try {
                process(imageDirectory);
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        });
    }

    private static void process(File imageDirectory) throws IOException {
        // Load images and configure the settings
        List<String> imageFiles = UtilIO.listSmartImages( imageDirectory.getPath(), true);
        var config = new Config(1000, 0.8, 2.0);
        if (!config.loadIntrinsic(imageDirectory.getPath()))
            config.guessIntrinsics(UtilImageIO.loadImageNotNull(imageFiles.get(0)));

        // Detect and match features
        var recog = new Recognizer(imageDirectory.getPath(), config);
        recog.detectFeat(imageFiles, config);
        recog.wrapFeat();
        recog.createModel();
        recog.createGraph();

        // Main Loop
        var views = new ArrayList<View>();
        var tracks = new ArrayList<Track>();

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
                var optimizer = new Optimizer(true);
                optimizer.initGraph(tracks, views);
                optimizer.wrapGraph(tracks, views, config);
                optimizer.process();
                optimizer.unwrapGraph(tracks, views, config);
            }
            System.out.printf("Registered view: %d\n", viewId);
        }

        // Global bundle adjustment
        var optimizer = new Optimizer(false);
        optimizer.initGraph(tracks, views);
        optimizer.wrapGraph(tracks, views, config);
        optimizer.process();
        optimizer.unwrapGraph(tracks, views, config);

        // Visualize
        SceneStructureMetric structure = optimizer.graph.getStructure();
        View.viewViews(tracks, views, config);
        optimizer.graph.viewCloud(imageFiles);
        SwingUtilities.invokeLater(() -> {
            // Opens a 3D viewer of sparse cloud. Use WASD keys and mouse to navigate
            BoofSwingUtil.visualizeCameras(structure, optimizer.graph.getViewer());
            ShowImages.showWindow(optimizer.graph.getViewer().getComponent(), "Refined Scene", true);
        });

        // Save output
        optimizer.graph.saveCloud();
        //CalibrationIO.save(config.intrinsic, "intrinsic.yaml");
    }

}