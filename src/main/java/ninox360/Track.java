package ninox360;

import java.util.List;

public class Track {
    int id;
    int length;
    List<Integer> camids;
    List<Integer> kpids;


    public Track(int id, int length, List<Integer> camids, List<Integer> kpids){
        this.id = id;
        this.camids = camids;
        this.kpids = kpids;
        this.length = length;
    }
}
