package org.firstinspires.ftc.teamcode.test.multiGluph;

/**
 * Created by kskrueger for Cybots Robotics on 3/10/18.
 */

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.BrownBird;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.BrownFrog;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.BrownSnake;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.GreyBird;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.GreyFrog;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.GreySnake;
import static org.firstinspires.ftc.teamcode.test.multiGluph.Ciphers.CurrentCipher;

public class SearchArray {
    private int column = 0;
    private int height = 0;

    private int vumarkColumn;
    private int vumarkGlyphColor;

    private boolean found = false;

    private boolean cipherCompatible = true;

    private boolean GreyFrogWorks = false;
    private boolean BrownFrogWorks = false;
    private boolean GreyBirdWorks = false;
    private boolean BrownBirdWorks = false;
    private boolean GreySnakeWorks = false;
    private boolean BrownSnakeWorks = false;

    public Ciphers.cipherType selectedCipher;

    public int[][] hopper = new int[][]{
            {0}, //fake bottom glyph to simulate solid surface below
            {0}, //1st glyph intaked (bottom when placing)
            {0}  //2nd glyph (top when placing)
    };

    public void setHopperGlyphs(int backGlyph, int frontGlyph) {
        hopper[1][0] = backGlyph;
        hopper[2][0] = frontGlyph;
    }

    public void setSingleGlyph(int backGlyph, int frontGlyph) {
        hopper[1][0] = backGlyph;
        hopper[2][0] = frontGlyph;
    }

    public void vumarkGlyph(int glyphColor, int column) {
        this.vumarkColumn = column;
        this.vumarkGlyphColor = glyphColor;

        if (GreyFrog[1][column]==vumarkGlyphColor) {
            GreyFrog[1][column] = 0;
            GreyFrogWorks = true;
        }
        if (BrownFrog[1][column]==vumarkGlyphColor) {
            BrownFrog[1][column] = 0;
            BrownFrogWorks = true;
        }
        if (GreyBird[1][column]==vumarkGlyphColor) {
            GreyBird[1][column] = 0;
            GreyBirdWorks = true;
        }
        if (BrownBird[1][column]==vumarkGlyphColor) {
            BrownBird[1][column] = 0;
            BrownBirdWorks = true;
        }
        if (GreySnake[1][column]==vumarkGlyphColor) {
            GreySnake[1][column] = 0;
            GreySnakeWorks = true;
        }
        if (BrownSnake[1][column]==vumarkGlyphColor) {
            BrownSnake[1][column] = 0;
            BrownSnakeWorks = true;
        }
    }

    /**
     * STEP 2 select cipher for which hopper load fits in
     */
    public void selectCipher3glyph(int[][] hopper) {
        if (searchSingle(GreyFrog,hopper) && GreyFrogWorks) {
            selectedCipher = Ciphers.cipherType.GreyFrog;

        }
        else if (searchSingle(BrownFrog,hopper) && BrownFrogWorks) {
            selectedCipher = Ciphers.cipherType.BrownFrog;
        }
        else if (searchSingle(GreyBird,hopper) && GreyBirdWorks) {
            selectedCipher = Ciphers.cipherType.GreyBird;
        }
        else if (searchSingle(BrownBird,hopper) && BrownBirdWorks) {
            selectedCipher = Ciphers.cipherType.BrownBird;
        }
        else if (searchSingle(GreySnake,hopper) && GreySnakeWorks) {
            selectedCipher = Ciphers.cipherType.GreySnake;
        }
        else if (searchSingle(BrownSnake,hopper) && BrownSnakeWorks) {
            selectedCipher = Ciphers.cipherType.BrownSnake;
        } else if (column==0) {
            selectedCipher = Ciphers.cipherType.Unknown;
            column = 2;
            height = 1;
            cipherCompatible = false;
        } else {
            selectedCipher = Ciphers.cipherType.Unknown;
            column = 0;
            height = 1;
            cipherCompatible = false;
        }

        if (cipherCompatible) {
            System.out.println("Selected Cipher: "+selectedCipher);
        } else {
            System.out.println("No Cipher Compatible, placed additional glyphs in "+column+" at "+height+" height");
        }

    }

    public void followCipher5glyph(int[][] hopper) {
        switch (selectedCipher) {
            case GreyFrog:
                CurrentCipher = GreyFrog;
                cipherCompatible = searchSingle(GreyFrog,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case BrownFrog:
                CurrentCipher = BrownFrog;
                cipherCompatible = searchSingle(BrownFrog,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case GreyBird:
                CurrentCipher = GreyBird;
                cipherCompatible = searchSingle(GreyBird,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case BrownBird:
                CurrentCipher = BrownBird;
                cipherCompatible = searchSingle(BrownBird,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case GreySnake:
                CurrentCipher = GreySnake;
                cipherCompatible = searchSingle(GreySnake,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case BrownSnake:
                CurrentCipher = BrownSnake;
                cipherCompatible = searchSingle(BrownSnake,hopper);
                System.out.println("Compatible: "+cipherCompatible);
                break;
            case Unknown:
                System.out.println("NOT Compatible");
                break;
        }

        if (!cipherCompatible) {
            if ((CurrentCipher[1][0]!=0)) {
                column = 0;
            } else if ((CurrentCipher[1][1]!=0)) {
                column = 1;
            } else if ((CurrentCipher[1][2]!=0)) {
                column = 2;
            }
        }

    }

    void printCipher () {
        switch (selectedCipher) {
            case GreyFrog:
                System.out.println(Arrays.deepToString(GreyFrog));
                break;
            case BrownFrog:
                System.out.println(Arrays.deepToString(BrownFrog));
                break;
            case GreyBird:
                System.out.println(Arrays.deepToString(GreyBird));
                break;
            case BrownBird:
                System.out.println(Arrays.deepToString(BrownBird));
                break;
            case GreySnake:
                System.out.println(Arrays.deepToString(GreySnake));
                break;
            case BrownSnake:
                System.out.println(Arrays.deepToString(BrownSnake));
                break;
        }
    }

    private boolean searchSingle(int[][] matrix, int[][] submatrix) {
        found = false;
        loopX: for (int x = 0; x < matrix.length - submatrix.length + 1; ++x)
            loopY: for (int y = 0; y < matrix[x].length - submatrix[0].length + 1; ++y)
            {
                for (int xx = 0; xx < submatrix.length; ++xx)
                    for (int yy = 0; yy < submatrix[0].length; ++yy)
                    {
                        if (matrix[x + xx][y + yy] != submatrix[xx][yy])
                        {
                            continue loopY;
                        }
                    }

                // Found the pattern in a cipher!
                found = true;
                column = y;
                height = x;
                System.out.println("Found at: " + x + " " + y);
                break loopX;
            }

        if (found) {
            matrix[height+1][column] = 0;
            matrix[height+2][column] = 0;
            System.out.println("Updated positions");
        }

        return found;
    }

    public int getColumn() {
        return column;
    }

    public int getHeight() {
        return height;
    }

}