package org.firstinspires.ftc.teamcode.util.multiGlyph.cipher;

class Ciphers {
    // 1 = grey
    // 2 = brown

    /*static int[][][] CipherOptions = new int[][][] {
            { //greyBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}
            },
            { //brownBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}
            },
            { //greyFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}
            },
            { //brownFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}
            },
            { //greySnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,2,1}, //1st row, left (0) to right (2)
                    {2,1,1}
            },
            { //brownSnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,1,2}, //1st row, left (0) to right (2)
                    {1,2,2}
            }
    };*/

    static int[][][] CipherOptions = new int[][][] {
            { //greyBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}, //2nd row
                    {2,1,2}, //3rd row
                    {1,2,1}  //top row of box
            },
            { //brownBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}, //2nd row
                    {1,2,1}, //3rd row
                    {2,1,2}  //top row of box
            },
            { //greyFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}, //2nd row
                    {1,2,1}, //3rd row
                    {2,1,2}  //top row of box
            },
            { //brownFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}, //2nd row
                    {2,1,2}, //3rd row
                    {1,2,1}  //top row of box
            },
            { //greySnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,2,1}, //1st row, left (0) to right (2)
                    {2,1,1}, //2nd row
                    {1,1,2}, //3rd row
                    {1,2,2}  //top row of box
            },
            { //brownSnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,1,2}, //1st row, left (0) to right (2)
                    {1,2,2}, //2nd row
                    {2,2,1}, //3rd row
                    {2,1,1}  //top row of box
            }
    };

    static int[][][] CipherOptionsOrgin = new int[][][] {
            { //greyBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}, //2nd row
                    {2,1,2}, //3rd row
                    {1,2,1}  //top row of box
            },
            { //brownBird
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}, //2nd row
                    {1,2,1}, //3rd row
                    {2,1,2}  //top row of box
            },
            { //greyFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,2,1}, //1st row, left (0) to right (2)
                    {2,1,2}, //2nd row
                    {1,2,1}, //3rd row
                    {2,1,2}  //top row of box
            },
            { //brownFrog
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,1,2}, //1st row, left (0) to right (2)
                    {1,2,1}, //2nd row
                    {2,1,2}, //3rd row
                    {1,2,1}  //top row of box
            },
            { //greySnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {2,2,1}, //1st row, left (0) to right (2)
                    {2,1,1}, //2nd row
                    {1,1,2}, //3rd row
                    {1,2,2}  //top row of box
            },
            { //brownSnake
                    {0,0,0}, //fake bottom row to simulate ground level
                    {1,1,2}, //1st row, left (0) to right (2)
                    {1,2,2}, //2nd row
                    {2,2,1}, //3rd row
                    {2,1,1}  //top row of box
            }
    };

    static boolean[] cipherCompatible = new boolean[]
            {
                    false, //greyBird
                    false, //brownBird
                    false, //greyFrog
                    false, //brownFrog
                    false, //greySnake
                    false  //brownSnake
            };

    public static void reset () {
        copyArray(CipherOptions,CipherOptionsOrgin);
    }

    private static void copyArray (int[][][] inputArray, int[][][] outputArray) {
        for (int z = 0; z <= 5; z ++) {
            for (int y = 0; y <= 2; y ++) {
                for (int x = 0; x <= 2; x ++) {
                    inputArray[z][y][x] = outputArray[z][y][x];
                }
            }
        }
    }
}

