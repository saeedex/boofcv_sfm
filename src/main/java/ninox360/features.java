package ninox360;

import boofcv.alg.descriptor.UtilFeature;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.FastAccess;
import java.util.ArrayList;
import java.util.List;

final class Feat {
    private final List<Point2D_F64> kps; // image coordinate
    private final DogArray<TupleDesc_F64> dscs;
    private final List<Integer> trackIds;

    public Feat(List<Point2D_F64> kps, DogArray<TupleDesc_F64> dscs, List<Integer> trackIds) {
        this.kps = kps;
        this.dscs = dscs;
        this.trackIds = trackIds;
    }

    public List<Point2D_F64> getkps() {
        return kps;
    }

    public DogArray<TupleDesc_F64> getdscs() {
        return dscs;
    }
    public List<Integer> getTrackIds() {
        return trackIds;
    }
}
public class features {
    public static Feat detect(GrayF32 image, Config config){
        // specify the image to process
        config.describer.detect(image);
        //System.out.println("Found Features: "+config.describer.getNumberOfFeatures());

        // store output
        List<Point2D_F64> kps = new ArrayList<>();
        DogArray<TupleDesc_F64> dscs = UtilFeature.createArray(config.describer, 100);
        List<Integer> idx = new ArrayList<>();

        for (int i = 0; i < config.describer.getNumberOfFeatures(); i++) {
            kps.add(config.describer.getLocation(i).copy());
            dscs.grow().setTo(config.describer.getDescription(i));
            idx.add(-1);
        }

        return new Feat(kps, dscs, idx);
    }

    public static FastAccess<AssociatedIndex> match(DogArray<TupleDesc_F64> descA, DogArray<TupleDesc_F64> descB, Config config){
        config.matcher.setSource(descA);
        config.matcher.setDestination(descB);
        config.matcher.associate();
        return config.matcher.getMatches();
    }
}