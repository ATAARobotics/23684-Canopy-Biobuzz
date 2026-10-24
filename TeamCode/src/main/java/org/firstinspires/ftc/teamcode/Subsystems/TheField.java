package org.firstinspires.ftc.teamcode.Subsystems;

import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.CompositeZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

public class TheField {

    public CompositeZone fieldBoundries;

    public TheField(){
        fieldBoundries = new CompositeZone(
                new CircleZone(new Point(94.2412109375, 1.8792977184057236), 10), // flower one
                new CircleZone(new Point(139.0126953125, 94.46230553090572), 10), // flower two
                new CircleZone(new Point(47.2587890625, 138.95742271840572), 10), // flower three
                new CircleZone(new Point(2.2109375, 47.203516468405724), 10), // flowery four

                new PolygonZone(new Point(47.2587890625,70.69472740590572), 5, 38.5), //hive red
                new PolygonZone(new Point(94.517578125,70.69472740590572), 5, 38.5), // hive blue

                new PolygonZone(new Point(72,0), 144, 3),
                new PolygonZone(new Point(0,72), 3, 144),
                new PolygonZone(new Point(144,72), 3, 144),
                new PolygonZone(new Point(72,144), 144, 3)
        );
    }
}
