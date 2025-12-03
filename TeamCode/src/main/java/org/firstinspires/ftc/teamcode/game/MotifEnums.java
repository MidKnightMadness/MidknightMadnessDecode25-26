package org.firstinspires.ftc.teamcode.game;

public class MotifEnums {
    public enum Motif {
        GPP(0, 1, 2),
        PGP(1, 0, 2),
        PPG(2, 0, 1),
        NONE(-1, -1, -1);
        int greenPosInd;
        int firstPurpleInd;
        int secondPurpleInd;

        Motif(int greenPosInd, int firstPurpleInd, int secondPurpleInd) {
            this.greenPosInd = greenPosInd;
            this.firstPurpleInd = firstPurpleInd;
            this.secondPurpleInd = secondPurpleInd;
        }


        public int getGreenPosInd(){
            return greenPosInd;
        }
        public int getFirstPurpleInd(){
            return firstPurpleInd;
        }
        public int getSecondPurpleInd(){
            return secondPurpleInd;
        }


    }
}
