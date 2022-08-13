package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.abst.scene.FeatureSceneRecognition;
import boofcv.abst.scene.SceneRecognition;
import boofcv.abst.scene.WrapFeatureToSceneRecognition;
import boofcv.abst.scene.nister2006.ConfigRecognitionNister2006;
import boofcv.alg.mvs.ColorizeMultiViewStereoResults;
import boofcv.core.image.LookUpColorRgbFormats;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.scene.FactorySceneRecognition;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.LookUpImageFilesByIndex;
import boofcv.io.image.UtilImageIO;
import boofcv.io.recognition.RecognitionIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU8;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point4D_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.DogArray_I32;
import org.ddogleg.struct.FastAccess;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Recognizer {
    ArrayList<Feat> featList;
    ArrayList<FeatureSceneRecognition.Features<TupleDesc_F64>> listRecFeat;
    FeatureSceneRecognition<TupleDesc_F64> recognizer;
    List<List<Integer>> conns;
    File outDir;

    public Recognizer(String imageDirectory, Config config){
        var configRecog = new ConfigRecognitionNister2006();
        configRecog.learningMinimumPointsForChildren.setFixed(20);
        this.recognizer = FactorySceneRecognition.createSceneNister2006(configRecog, config.describer::createDescription);
        this.outDir = new File(imageDirectory, "example_recognition");
    }
    public void detectFeat(List<String> imageFiles, Config config){
        featList = new ArrayList<>();
        for (String imageFile : imageFiles){
            BufferedImage img = UtilImageIO.loadImageNotNull(imageFile);
            featList.add(features.detect(ConvertBufferedImage.convertFrom(img, (GrayF32)null), config));
        }
    }
    public void createCraph(){
        conns = new ArrayList<>();
        conns.add(new ArrayList<>());
        var matches = new DogArray<>(SceneRecognition.Match::new);

        for (int imageIdx = 1; imageIdx < featList.size(); imageIdx++) {
            int _imageIdx = imageIdx;
            recognizer.query(
                    /*query*/ listRecFeat.get(imageIdx),
                    /*filter*/ ( id ) -> (_imageIdx - Integer.parseInt(id)) > 20,
                    /*limit*/ 5, /*found matches*/ matches);

            List<Integer> conviewIds = new ArrayList<>();
            conviewIds.add(imageIdx-1); // match with previous by default
            for (var m : matches.toList()) {
                conviewIds.add(Integer.parseInt(m.id));
            }
            conns.add(conviewIds);
        }
    }
    public void wrapFeat(){
        // Put feature information into a format scene recognition understands
        this.listRecFeat = new ArrayList<FeatureSceneRecognition.Features<TupleDesc_F64>>();
        for (Feat feat : featList) {
            List<Point2D_F64> pixels = feat.getkps();
            FastAccess<TupleDesc_F64> descs = feat.getdscs();
            this.listRecFeat.add(new FeatureSceneRecognition.Features<>() {
                @Override
                public Point2D_F64 getPixel(int index) {
                    return pixels.get(index);
                }

                @Override
                public TupleDesc_F64 getDescription(int index) {
                    return descs.get(index);
                }

                @Override
                public int size() {
                    return pixels.size();
                }
            });
        }
    }
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

    public void saveModel(){
        // This saves the model with the image database to disk
        // todo: save model
        System.out.println("Saving model");
        BoofMiscOps.profile(() -> RecognitionIO.saveFeatureToScene(
                (WrapFeatureToSceneRecognition<GrayU8, ?>)this.recognizer, this.outDir), "");
    }
}
