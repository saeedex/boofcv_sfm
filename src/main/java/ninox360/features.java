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

final class feat {
    private final List<Point2D_F64> kps;
    private final DogArray<TupleDesc_F64> dscs;
    private final List<Integer> trackids;

    public feat(List<Point2D_F64> kps, DogArray<TupleDesc_F64> dscs, List<Integer> trackids) {
        this.kps = kps;
        this.dscs = dscs;
        this.trackids = trackids;
    }

    public List<Point2D_F64> getkps() {
        return kps;
    }
    public DogArray<TupleDesc_F64> getdscs() {
        return dscs;
    }
    public List<Integer> gettrackids() {
        return trackids;
    }
}
public class features {
    public static feat detect(GrayF32 image, Config config){
        // specify the image to process
        config.describer.detect(image);
        //System.out.println("Found Features: "+config.describer.getNumberOfFeatures());

        // store output
        List<Point2D_F64> points = new ArrayList<>();
        DogArray<TupleDesc_F64> desc = UtilFeature.createArray(config.describer, 100);
        List<Integer> idx = new ArrayList<>();

        for (int i = 0; i < config.describer.getNumberOfFeatures(); i++) {
            points.add(config.describer.getLocation(i).copy());
            desc.grow().setTo(config.describer.getDescription(i));
            idx.add(-1);
        }

        return new feat(points, desc, idx);
    }

    public static FastAccess<AssociatedIndex> match(DogArray<TupleDesc_F64> descA, DogArray<TupleDesc_F64> descB, Config config){
        config.matcher.setSource(descA);
        config.matcher.setDestination(descB);
        config.matcher.associate();
        FastAccess<AssociatedIndex> idxpair = config.matcher.getMatches();
        return idxpair;
    }

    public static void map(List<Track> tracks, List<Camera> cameras, FastAccess<AssociatedIndex> idxpair){
        int camid = cameras.size() - 1;
        int pcamid = camid - 1;

        for (int i = 0; i < idxpair.size; i++) {
            int ptrkid = cameras.get(pcamid).trackids.get(idxpair.get(i).dst);

            if (ptrkid == -1) {
                // Create new tracks
                List<Integer> camids = new ArrayList<>();
                List<Integer> kpids = new ArrayList<>();
                camids.add(pcamid);
                kpids.add(idxpair.get(i).dst);
                tracks.add(new Track(tracks.size(), 1, camids, kpids));
                int trkid = tracks.size() - 1;
                cameras.get(pcamid).trackids.set(idxpair.get(i).dst, trkid);
                ptrkid = trkid;
            }

            // Update existing tracks
            tracks.get(ptrkid).camids.add(camid);
            tracks.get(ptrkid).kpids.add(idxpair.get(i).src);
            tracks.get(ptrkid).length += 1;
            cameras.get(camid).trackids.set(idxpair.get(i).src, ptrkid);
        }
    }
}