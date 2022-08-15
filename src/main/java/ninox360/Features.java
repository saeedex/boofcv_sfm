package ninox360;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.alg.descriptor.UtilFeature;
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
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
    /**
     * detects and describes features inside an image
     * @param image input grayscale image
     * @param config configuration
     * @return ImgFeats: all the feratures detected inside an image
     */
    public static ImgFeats detect(GrayF32 image, Config config){
        // specify the image to process
        config.describer.detect(image);

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

    /**
     * associates feature descriptors
     * @param src source descriptor set
     * @param dst destination descriptor
     * @param config configuration
     * @return associated feature index pairs
     */
    public static FastAccess<AssociatedIndex> match(DogArray<TupleDesc_F64> src, DogArray<TupleDesc_F64> dst, Config config){
        AssociateDescription<TupleDesc_F64> matcher =
                FactoryAssociation.greedy(new ConfigAssociateGreedy(true, config.matcherThreshold), config.scorer);
        matcher.setSource(src);
        matcher.setDestination(dst);
        matcher.associate();
        return matcher.getMatches();
    }
}