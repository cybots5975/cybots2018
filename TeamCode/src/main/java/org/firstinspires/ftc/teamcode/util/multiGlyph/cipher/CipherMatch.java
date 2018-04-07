package org.firstinspires.ftc.teamcode.util.multiGlyph.cipher;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.Ciphers.CipherOptions;
import static org.firstinspires.ftc.teamcode.util.multiGlyph.cipher.Ciphers.cipherCompatible;

public class CipherMatch {
    private int column = 0;
    private int height = 0;

    public int cipherChosen;

    public enum GlyphColor {
        GREY, BROWN, UNKNOWN
    }

    boolean cipherValid = false;

    public int[][] hopper = new int[][]{
            {0}, //fake bottom glyph to simulate solid surface below
            {1}, //1st glyph intaked (bottom when placing)
            {2}  //2nd glyph (top when placing)
    };

    public void setHopperGlyphs(int backGlyph, int frontGlyph) {
        hopper[1][0] = backGlyph;
        hopper[2][0] = frontGlyph;
    }

    public void setSingleGlyph(int backGlyph, int frontGlyph) {
        hopper[1][0] = backGlyph;
    }

    public void setLoad1 (GlyphColor glyphColor1, GlyphColor glyphColor2, int vumarkColumn) {

    }

    public void setLoad1 (GlyphColor glyphColor1, int vumarkColumn) {

    }

    public void setLoad2 (GlyphColor glyphColor1, GlyphColor glyphColor2) {

    }

    public void calculate () {

    }

    public void vumarkGlyph(int glyphColor, int column) {
        this.column = column;

        for (int i = 0; i <= 5; i ++) {
            if (CipherOptions[i][1][column] == glyphColor) {
                cipherChosen = i;

                cipherCompatible[i] = true;

                CipherOptions[i][1][column] = 0;
            } else {
                cipherCompatible[i] = false;
            }
        }

        System.out.println("Vumark glyph scored in "+glyphColumn(getColumn())+" column");
    }

    void vumarkGlyph(int glyphColor, int glyphColor2, int column) {
        this.column = column;

        for (int i = 0; i <= 5; i ++) {
            if (CipherOptions[i][1][column] == glyphColor
                    && CipherOptions[i][2][column] == glyphColor2) {
                cipherChosen = i;

                cipherCompatible[i] = true;

                CipherOptions[i][1][column] = 0;
                CipherOptions[i][2][column] = 0;
            } else {
                cipherCompatible[i] = false;
            }
        }

        System.out.println("Vumark and bonus glyph scored in "+glyphColumn(getColumn())+" column");
    }

    public void vumarkGlyph(int[][] hopper, int column) {
        this.column = column;

        for (int i = 0; i <= 5; i ++) {
            if (CipherOptions[i][1][column] == hopper[1][0]
                    && CipherOptions[i][2][column] == hopper[2][0]) {
                cipherChosen = i;

                cipherCompatible[i] = true;

                CipherOptions[i][1][column] = 0;
                CipherOptions[i][2][column] = 0;
            } else {
                cipherCompatible[i] = false;
            }

        }

        System.out.println("Vumark and bonus glyph scored in "+glyphColumn(getColumn())+" column");
    }

    public void search(int[][] hopper) {

        for (int i = 0; i <= 5; i ++) {
            if (searchSingle(CipherOptions[i],hopper) && cipherCompatible[i]) {
                cipherChosen = i;

                cipherCompatible[i] = true;

                CipherOptions[i][height+1][column] = 0;
                CipherOptions[i][height+2][column] = 0;
            } else {
                cipherCompatible[i] = false;
            }
        }

        System.out.println("Glyphs scored in "+glyphColumn(column)+" column at "+glyphHeight(height)+" height");
    }

    public void printCipher() {
        for (int i = 0; i <= 5; i ++) {
            if (cipherCompatible[i]) {
                System.out.println(cipherNumber(i));
                System.out.println(Arrays.deepToString(CipherOptions[i]));
            }
        }
    }

    public boolean checkCipherValid() {
        cipherValid = false;
        for (int i = 0; i <=5; i ++) {
            if (cipherCompatible[i]) {
                cipherValid = true;
            }
        }
        return cipherValid;
    }

    private boolean searchSingle(int[][] matrix, int[][] submatrix) {
        boolean found = false;
        loopX: for (int y = 0; y < matrix.length - submatrix.length + 1; ++y)
            loopY: for (int x = 0; x < matrix[y].length - submatrix[0].length + 1; ++x)
            {
                for (int yy = 0; yy < submatrix.length; ++yy)
                    for (int xx = 0; xx < submatrix[0].length; ++xx)
                    {
                        if (matrix[y + yy][x + xx] != submatrix[yy][xx])
                        {
                            continue loopY;
                        }
                    }

                // Found the pattern in a cipher!
                found = true;
                column = x;
                height = y;
                System.out.println("Found at: " + x + " column, " + y +" height");
                save(x,y);
                break loopX;
            }

            if (found) {
                matrix[height+1][column] = 0;
                matrix[height+2][column] = 0;
                //System.out.println("Updated positions");
            }

            return found;
    }

    public int getColumn() {
        return column;
    }

    public int getHeight() {
        return height;
    }

    public String cipherNumber(int number) {
        switch (number) {
            case 0:
                return "Grey Bird";
            case 1:
                return "Brown Bird";
            case 2:
                return "Grey Frog";
            case 3:
                return "Brown Frog";
            case 4:
                return "Grey Snake";
            case 5:
                return "Brown Snake";
        }
        return "None";
    }

    String glyphColor(int color) {
        if (color==1) {
            return "Grey";
        } else {
            return "Brown";
        }
    }

    public String glyphColors(int[][] hopper) {
        return glyphColor(hopper[1][0])+"_"+glyphColor(hopper[2][0]);
    }

    public String glyphColumn(int column) {
        if (column==0) {
            return "Left";
        } else if (column==1) {
            return "Center";
        } else {
            return "Right";
        }
    }

    private String glyphHeight(int height) {
        if (getHeight()==0) {
            return  "Low";
        } else if (getHeight()==1) {
            return  "Mid";
        } else if (getHeight()==2) {
            return "High";
        } else {
            return "hmm?";
        }
    }

    private void save (int column, int height) {
        this.height = height;
        this.column = column;
    }

}