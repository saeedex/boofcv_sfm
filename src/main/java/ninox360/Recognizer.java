package ninox360;

import boofcv.abst.scene.FeatureSceneRecognition;
import boofcv.abst.scene.SceneRecognition;
import boofcv.abst.scene.nister2006.ConfigRecognitionNister2006;
import boofcv.factory.scene.FactorySceneRecognition;
import boofcv.struct.feature.TupleDesc_F64;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.FastAccess;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Recognizer {
    ArrayList<FeatureSceneRecognition.Features<TupleDesc_F64>> listRecFeat;
    FeatureSceneRecognition<TupleDesc_F64> recognizer;

    public Recognizer(ArrayList<Feat> featList, Config config){
        var configRecog = new ConfigRecognitionNister2006();
        configRecog.learningMinimumPointsForChildren.setFixed(20);

        this.recognizer = FactorySceneRecognition.createSceneNister2006(configRecog, config.describer::createDescription);

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

    public void createDatabase(ArrayList<Feat> featList){
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

    public boolean query(int imageIdx, int loopid){
        int _imageIdx = imageIdx;
        var matches = new DogArray<>(SceneRecognition.Match::new);
        recognizer.query(
                /*query*/ listRecFeat.get(imageIdx),
                /*filter*/ ( id ) -> (_imageIdx - Integer.parseInt(id)) > 60,
                /*limit*/ 5, /*found matches*/ matches);

        if (matches.toList().get(0).id != null) {
            loopid = Integer.parseInt(matches.toList().get(0).id);
            System.out.printf("Loop: %d\n", loopid);
            return true;
        }
        else return false;

    }
}
