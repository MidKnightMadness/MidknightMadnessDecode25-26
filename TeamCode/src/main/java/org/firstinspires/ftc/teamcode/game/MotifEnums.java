package org.firstinspires.ftc.teamcode.game;

public class MotifEnums {
    public enum Motif {
        GPP(0, 1, 2, BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
        PGP(1, 0, 2, BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
        PPG(2, 0, 1, BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN),
        NONE(-1, -1, -1, BallColor.NONE, BallColor.NONE, BallColor.NONE);
        int greenPosInd;
        int firstPurpleInd;
        int secondPurpleInd;
        BallColor first;
        BallColor second;
        BallColor third;

        Motif(int greenPosInd, int firstPurpleInd, int secondPurpleInd, BallColor first, BallColor second, BallColor third) {
            this.greenPosInd = greenPosInd;
            this.firstPurpleInd = firstPurpleInd;
            this.secondPurpleInd = secondPurpleInd;
            this.first = first;
            this.second = second;
            this.third = third;
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

        public BallColor getBallColorFromIndex(int i){
             if(i == 0){
                 return first;
             }
             else if(i == 1){
                 return second;
             }
             else if(i == 2){
                 return third;
             }
             return null;
        }

    }
}
