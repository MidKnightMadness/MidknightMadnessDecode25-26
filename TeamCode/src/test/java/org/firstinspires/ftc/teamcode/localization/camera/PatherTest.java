package org.firstinspires.ftc.teamcode.localization.camera;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;

class PatherTest {
    @Test
    void generatePermutations() {
        int n = 5, k = 3;
        ArrayList<int[]> permutations = Pather.generatePermutations(n, k);
        for (int[] arr : permutations) {
            System.out.println(Arrays.toString(arr));
        }
        System.out.println(permutations.size());
    }

}