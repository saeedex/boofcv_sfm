package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.alg.descriptor.UtilFeature;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import lombok.Data;
import org.ddogleg.struct.DogArray;
import org.ddogleg.struct.DogArray_I32;
import org.ddogleg.struct.FastAccess;
import java.util.ArrayList;
import java.util.List;

/**
 * All the features detected inside an image.
 *
 * @param kps      Key points. Pixel coordinates of image features
 * @param dscs     Descriptors of key points
 * @param trackIds Which track this feature is an observation of. -1 means unassigned
 */
record ImgFeats(List<Point2D_F64> kps, DogArray<TupleDesc_F64> dscs, DogArray_I32 trackIds) {}

public class Features {
    public static ImgFeats detect(GrayF32 image, Config config){
        // specify the image to process
        config.describer.detect(image);
        System.out.println("Found Features: " + config.describer.getNumberOfFeatures());

        // store output
        var kps = new ArrayList<Point2D_F64>();
        DogArray<TupleDesc_F64> dscs = UtilFeature.createArray(config.describer, 100);
        var idx = new DogArray_I32();

        idx.resize(config.describer.getNumberOfFeatures(), -1);
        for (int i = 0; i < config.describer.getNumberOfFeatures(); i++) {
            kps.add(config.describer.getLocation(i).copy());
            dscs.grow().setTo(config.describer.getDescription(i));
        }

        return new ImgFeats(kps, dscs, idx);
    }

    public static FastAccess<AssociatedIndex> match(DogArray<TupleDesc_F64> descA, DogArray<TupleDesc_F64> descB, Config config){
        AssociateDescription<TupleDesc_F64> matcher =
                FactoryAssociation.greedy(new ConfigAssociateGreedy(true, config.matcherThreshold), config.scorer);
        matcher.setSource(descA);
        matcher.setDestination(descB);
        matcher.associate();
        return matcher.getMatches();
    }
}