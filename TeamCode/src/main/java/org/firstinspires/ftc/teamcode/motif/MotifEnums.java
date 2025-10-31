package org.firstinspires.ftc.teamcode.motif;

public class MotifEnums {
    public enum Motif {
        GPP,
        PGP,
        PPG,
        NONE;

        public int getGreenPosition() {
            if (this == Motif.GPP) return 0;
            if (this == Motif.PGP) return 1;
            if (this == Motif.PPG) return 2;
            return -1;
        }
    }
}
