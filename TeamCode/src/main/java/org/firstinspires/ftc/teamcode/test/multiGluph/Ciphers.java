package org.firstinspires.ftc.teamcode.test.multiGluph;

/**
 * Created by kskrueger for Cybots Robotics on 3/10/18.
 */

class Ciphers {
    // 1 = grey
    // 2 = brown

    public enum cipherType {Unknown, GreyFrog, BrownFrog, GreySnake, BrownSnake, GreyBird, BrownBird}

    static int[][] GreyFrog = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {1,2,1}, //1st row, left (0) to right (2)
            {2,1,2}, //2nd row
            {1,2,1}, //3rd row
            {2,1,2}  //top row of box
    };

    static int[][] BrownFrog = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {2,1,2}, //1st row, left (0) to right (2)
            {1,2,1}, //2nd row
            {2,1,2}, //3rd row
            {1,2,1}  //top row of box
    };

    static int[][] GreySnake = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {2,2,1}, //1st row, left (0) to right (2)
            {2,1,1}, //2nd row
            {1,1,2}, //3rd row
            {1,2,2}  //top row of box
    };

    static int[][] BrownSnake = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {1,1,2}, //1st row, left (0) to right (2)
            {1,2,2}, //2nd row
            {2,2,1}, //3rd row
            {2,1,1}  //top row of box
    };

    static int[][] GreyBird = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {1,2,1}, //1st row, left (0) to right (2)
            {2,1,2}, //2nd row
            {2,1,2}, //3rd row
            {1,2,1}  //top row of box
    };

    static int[][] BrownBird = new int[][]{
            {0,0,0}, //fake bottom row to simulate ground level
            {2,1,2}, //1st row, left (0) to right (2)
            {1,2,1}, //2nd row
            {1,2,1}, //3rd row
            {2,1,2}  //top row of box
    };

    static int[][] CurrentCipher = new int[][]{
            {0,0,0}, //empty start
            {0,0,0}, //empty start
            {0,0,0}, //empty start
            {0,0,0}, //empty start
            {0,0,0} //empty start
    };
}