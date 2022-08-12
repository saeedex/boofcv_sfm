package ninox360;

import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.abst.scene.FeatureSceneRecognition;
import boofcv.abst.scene.nister2006.ConfigRecognitionNister2006;
import boofcv.factory.scene.FactorySceneRecognition;
import boofcv.gui.BoofSwingUtil;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.struct.FastAccess;

import javax.swing.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
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

        // Initialize scene recognition
        var featList = new ArrayList<Feat>();
        for (String imageFile : imageFiles){
            BufferedImage img = UtilImageIO.loadImageNotNull(imageFile);
            featList.add(features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config));
        }
        Recognizer recog = new Recognizer(featList, config);
        recog.createDatabase(featList);


        // Main Loop
        for (String imageFile : imageFiles){
            // add new view (detect features)
            int viewId = views.size();
            //System.out.println(viewId);
            views.add(new View(viewId, imageFile, config, featList.get(viewId)));

            if (viewId != 0) {
                // map existing tracks
                // create new tracks
                int mid = viewId - 1;
                int loopid = 0;
                views.get(viewId).addConnection(mid, views.get(mid).dscs, config);
                //sif (recog.query(viewId, loopid) ) views.get(viewId).addConnection(loopid, views.get(loopid).dscs, config); //loop closure here

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