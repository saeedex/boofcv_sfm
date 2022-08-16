package ninox360;

import boofcv.abst.scene.FeatureSceneRecognition;
import boofcv.abst.scene.SceneRecognition;
import boofcv.abst.scene.nister2006.ConfigRecognitionNister2006;
import boofcv.abst.scene.nister2006.FeatureSceneRecognitionNister2006;
import boofcv.factory.scene.FactorySceneRecognition;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.io.recognition.RecognitionIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.FastAccess;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Recognizer class: interface to detect most similar images in a sequence. More details at {@link SceneRecognition}.
 */
public class Recognizer {
    /**
     * List of detected features {@link ImgFeats} in all images in a sequence.
     */
    ArrayList<ImgFeats> featList;
    /**
     * More specialized version of Recognizer where it is assumed the input is composed of image features
     * that have been detected sparsely at different pixel coordinates.
     */
    FeatureSceneRecognitionNister2006<TupleDesc_F64> recognizer;
    /**
     * List of detected features {@link ImgFeats} in a format Recognizer understands.
     */
    private ArrayList<FeatureSceneRecognition.Features<TupleDesc_F64>> listRecFeat;
    /**
     * List of list of {@link Connection}
     */
    List<List<Integer>> conns;
    /**
     * Output directory where generated model is saved.
     */
    File outDir;

    public Recognizer(String imageDirectory, Config config){
        var configRecog = new ConfigRecognitionNister2006();
        configRecog.learningMinimumPointsForChildren.setFixed(20);
        this.recognizer = FactorySceneRecognition.createSceneNister2006(configRecog, config.describer::createDescription);
        this.outDir = new File(imageDirectory, "recognition");
    }

    /**
     * Detects features {@link ImgFeats} in all images
     * @param imageFiles list of image file names
     * @param config configuration
     */
    public void detectFeat(List<String> imageFiles, Config config){
        System.out.println("Detecting features:");
        featList = new ArrayList<>();
        for (int i = 0; i < imageFiles.size(); i++) {
            BufferedImage img = UtilImageIO.loadImageNotNull(imageFiles.get(i));
            featList.add(Features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config));
            System.out.printf("  Image[%3d] features.size=%d\n", i, config.describer.getNumberOfFeatures());
        }
        wrapFeat();
    }

    /**
     * Wraps list of features {@link ImgFeats} to a format {@link Recognizer} understands
     */
    public void wrapFeat(){
        // Put feature information into a format scene recognition understands
        this.listRecFeat = new ArrayList<>();
        for (ImgFeats feat : featList) {
            List<Point2D_F64> pixels = feat.kps();
            FastAccess<TupleDesc_F64> descs = feat.dscs();
            this.listRecFeat.add(new FeatureSceneRecognition.Features<>() {
                @Override public Point2D_F64 getPixel(int index)         {return pixels.get(index);}
                @Override public TupleDesc_F64 getDescription(int index) {return descs.get(index);}
                @Override public int size()                              {return pixels.size();}
            });
        }
    }

    /**
     * Generates image connectivity graph.
     * For all images in a sequence it describes how is that image is connected with other images.
     */
    public void createGraph(){
        conns = new ArrayList<>();
        conns.add(new ArrayList<>());
        var matches = new DogArray<>(SceneRecognition.Match::new);
        for (int imageIdx = 1; imageIdx < featList.size(); imageIdx++) {
            int _imageIdx = imageIdx;

            recognizer.query(/*query*/ listRecFeat.get(imageIdx),/*filter*/ ( id ) -> (_imageIdx - Integer.parseInt(id)) > 20,/*limit*/ 5, /*found matches*/ matches);
            List<Integer> conviewIds = new ArrayList<>();
            conviewIds.add(imageIdx-1); // match with previous by default

            for (var m : matches.toList()) {
                conviewIds.add(Integer.parseInt(m.id));
            }
            conns.add(conviewIds);
        }

    }

    /**
     * Generates a scene recognition model
     */
    public void createModel(){
        // Pass image information in as an iterator that it understands.
        this.recognizer.learnModel(new Iterator<>() {
            int imageIndex = 0;

            @Override public boolean hasNext() {return imageIndex < featList.size();}

            @Override public FeatureSceneRecognition.Features<TupleDesc_F64> next() {
                return listRecFeat.get(imageIndex++);
            }
        });

        System.out.println("Creating database");
        for (int imageIdx = 0; imageIdx < featList.size(); imageIdx++) {
            // Note that image are assigned a name equal to their index
            recognizer.addImage(imageIdx + "", listRecFeat.get(imageIdx));
        }
    }

    /**
     * Saves generated scene recognition model.
     */
    public void saveModel(){
        // This saves the model with the image database to disk
        System.out.println("Saving model");
        RecognitionIO.saveNister2006(this.recognizer, this.outDir);
    }

    /**
     * Loads generated scene recognition model.
     * @return true if loaded
     */
    public boolean loadModel(){
        boolean flag = false;
        if(this.outDir.exists()) {
            System.out.println("Loading model");
            RecognitionIO.loadNister2006(this.outDir, this.recognizer);
            flag = true;
        }
        return flag;
    }
}
