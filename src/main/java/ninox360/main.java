package ninox360;

import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.image.GrayF32;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.FastAccess;
import org.ejml.data.DMatrixRMaj;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * saeed 2022-08-01: load and detect features in a directory
 *
 */
public class main {
    public static void main(String[] args) {
        String imageDirectory = "/Users/saad/Documents/slam/pyDex3D/dataset/";
        List<String> imageFiles = UtilIO.listImages( imageDirectory, true);
        List<Camera> cameras = new ArrayList<>();
        List<Track> tracks = new ArrayList<>();
        var gui = new ListDisplayPanel();
        Se3_F64 kpose;

        // Config
        Config config = new Config(1000, 0.1, 0.8);
        config.getintrinsic(UtilImageIO.loadImageNotNull(imageFiles.get(0)));

        for (String imageFile : imageFiles){
            // load images
            BufferedImage img = UtilImageIO.loadImageNotNull(imageFile);

            // detect features
            feat cfeat = features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config);
            Camera camera = new Camera(cameras.size(), imageFile, img, cfeat.getkps(), cfeat.getdscs(), cfeat.gettrackids());
            cameras.add(camera);

            int id = cameras.size() - 1;
            if (id != 0) {
                // match features
                FastAccess<AssociatedIndex> idxpair = features.match(cameras.get(id).dscs, cameras.get(id - 1).dscs, config);

                // mapping
                features.map(tracks, cameras, idxpair);

                if (!config.init)
                    kpose = pose.init(tracks, cameras, config);
                    break;

            }
        }
        // Visualization
//        for ( Camera camera : cameras){
//            camera.viewkps();
//            camera.viewtracks();
//            gui.addImage(camera.img, camera.file);
//        }
//        ShowImages.showWindow(gui,"detected features", true);
    }
}