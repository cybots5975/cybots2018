package org.firstinspires.ftc.teamcode.util.multiGlyph.cipher;

public class Test2 {
    private static CipherMatch cipherTest = new CipherMatch();

    private static int cipherCount = 0;
    private static int totalCount = 0;

    private static int[][] load1 = new int[][]{
            {0}, //1st glyph preload glyph
            {1},  //2nd glyph intaked
            {1}  //2nd glyph intaked
    };

    private static int[][] load2 = new int[][]{
            {0}, //fake bottom glyph to simulate solid surface below
            {2}, //1st glyph intaked (bottom when placing)
            {2}  //2nd glyph (top when placing)
    };

    private static int[][] load3 = new int[][]{
            {0}, //fake bottom glyph to simulate solid surface below
            {2}, //1st glyph intaked (bottom when placing)
            {2}  //2nd glyph (top when placing)
    };

    public static void main(String[] args) {
        //count6glyph();
        complete5glyph();

    }

    private static void complete4glyph() {
        cipherTest.vumarkGlyph(load1,0);

        //cipherTest.printCipher();

        //System.out.println(cipherTest.checkCipherValid());

        cipherTest.search(load2);

        cipherTest.printCipher();

        System.out.println(cipherTest.checkCipherValid());
        System.out.println(cipherTest.cipherNumber(cipherTest.cipherChosen));

    }

    private static void complete5glyph() {
        cipherTest.vumarkGlyph(2,1);

        cipherTest.search(load2);

        cipherTest.printCipher();
        System.out.println(cipherTest.checkCipherValid());

        cipherTest.search(load3);

        cipherTest.printCipher();
        System.out.println(cipherTest.checkCipherValid());
    }

    private static void count4glyph() {
        for (int vumarkColumn = 0; vumarkColumn <=2; vumarkColumn ++) {
            for (int firstLoad = 0; firstLoad <= 3; firstLoad ++) {
                for (int secondLoad = 0; secondLoad <= 3; secondLoad ++) {
                    cipherTest.vumarkGlyph(load[firstLoad],vumarkColumn);
                    cipherTest.search(load[secondLoad]);

                    System.out.println(cipherTest.glyphColumn(vumarkColumn)
                            +", "+cipherTest.glyphColors(load[firstLoad])
                            +", "+cipherTest.glyphColors(load[secondLoad])
                            +", "+cipherTest.checkCipherValid()
                            +", "+cipherTest.cipherNumber(cipherTest.cipherChosen));

                    totalCount += 1;

                    if (cipherTest.checkCipherValid()) {
                        cipherCount += 1;
                    }
                    Ciphers.reset();
                }
            }
        }

        System.out.println("Total Count: "+totalCount);
        System.out.println("Compatible Count: "+cipherCount);
    }

    private static void count5glyph() {
        for (int vumarkColumn = 0; vumarkColumn <=2; vumarkColumn ++) {

            for (int vumarkGlyph = 1; vumarkGlyph <= 2; vumarkGlyph ++) {
                for (int firstLoad = 0; firstLoad <= 3; firstLoad++) {
                    for (int secondLoad = 0; secondLoad <= 3; secondLoad++) {
                        cipherTest.vumarkGlyph(vumarkGlyph, vumarkColumn);
                        cipherTest.search(load[firstLoad]);
                        cipherTest.search(load[secondLoad]);
                        System.out.println(cipherTest.glyphColumn(vumarkColumn)
                                +", "+cipherTest.glyphColor(vumarkGlyph)
                                +", "+cipherTest.glyphColors(load[firstLoad])
                                +", "+cipherTest.glyphColors(load[secondLoad])
                                +", "+cipherTest.checkCipherValid()
                                +", "+cipherTest.cipherNumber(cipherTest.cipherChosen));

                        totalCount += 1;
                        if (cipherTest.checkCipherValid()) {
                            cipherCount += 1;
                        }
                        Ciphers.reset();
                    }
                }
            }
        }

        System.out.println("Total Count: "+totalCount);
        System.out.println("Compatible Count: "+cipherCount);
    }

    private static void count6glyph() {
        for (int vumarkColumn = 0; vumarkColumn <=2; vumarkColumn ++) {
            for (int firstLoad = 0; firstLoad <= 3; firstLoad++) {
                for (int secondLoad = 0; secondLoad <= 3; secondLoad++) {
                    for (int thirdLoad = 0; thirdLoad <= 3; thirdLoad++) {
                        cipherTest.vumarkGlyph(load[firstLoad], vumarkColumn);
                        cipherTest.search(load[secondLoad]);
                        cipherTest.search(load[thirdLoad]);

                        System.out.println(cipherTest.glyphColumn(vumarkColumn)
                                + ", " + cipherTest.glyphColors(load[firstLoad])
                                + ", " + cipherTest.glyphColors(load[secondLoad])
                                + ", " + cipherTest.glyphColors(load[thirdLoad])
                                + ", " + cipherTest.checkCipherValid()
                                + ", " + cipherTest.cipherNumber(cipherTest.cipherChosen));

                        totalCount += 1;

                        if (cipherTest.checkCipherValid()) {
                            cipherCount += 1;
                        }
                        Ciphers.reset();
                    }
                }
            }
        }

        System.out.println("Total Count: "+totalCount);
        System.out.println("Compatible Count: "+cipherCount);
    }

    private static void count11glyph() {
        for (int vumarkColumn = 0; vumarkColumn <=2; vumarkColumn ++) {
            for (int vumarkGlyph = 1; vumarkGlyph <= 2; vumarkGlyph ++) {
                for (int firstLoad = 0; firstLoad <= 3; firstLoad++) {
                    for (int secondLoad = 0; secondLoad <= 3; secondLoad++) {
                        for (int thirdLoad = 0; thirdLoad <= 3; thirdLoad++) {
                            for (int fourthLoad = 0; fourthLoad <= 3; fourthLoad++) {
                                for (int fifthLoad = 0; fifthLoad <= 3; fifthLoad++) {
                                    cipherTest.vumarkGlyph(vumarkGlyph, vumarkColumn);
                                    cipherTest.search(load[firstLoad]);
                                    cipherTest.search(load[secondLoad]);
                                    cipherTest.search(load[thirdLoad]);
                                    cipherTest.search(load[fourthLoad]);
                                    cipherTest.search(load[fifthLoad]);
                                    System.out.println(cipherTest.glyphColumn(vumarkColumn)
                                            + ", " + cipherTest.glyphColor(vumarkGlyph)
                                            + ", " + cipherTest.glyphColors(load[firstLoad])
                                            + ", " + cipherTest.glyphColors(load[secondLoad])
                                            + ", " + cipherTest.glyphColors(load[thirdLoad])
                                            + ", " + cipherTest.glyphColors(load[fourthLoad])
                                            + ", " + cipherTest.glyphColors(load[fifthLoad])
                                            + ", " + cipherTest.checkCipherValid()
                                            + ", " + cipherTest.cipherNumber(cipherTest.cipherChosen));

                                    totalCount += 1;
                                    if (cipherTest.checkCipherValid()) {
                                        cipherCount += 1;
                                    }
                                    Ciphers.reset();
                                }
                            }
                        }
                    }
                }
            }
        }

        System.out.println("Total Count: "+totalCount);
        System.out.println("Compatible Count: "+cipherCount);
    }

    private static void count12glyph() {
        for (int vumarkColumn = 0; vumarkColumn <=2; vumarkColumn ++) {
            for (int firstLoad = 0; firstLoad <= 3; firstLoad ++) {
                for (int secondLoad = 0; secondLoad <= 3; secondLoad ++) {
                    for (int thirdLoad = 0; thirdLoad <= 3; thirdLoad ++) {
                        for (int fourthLoad = 0; fourthLoad <= 3; fourthLoad ++) {
                            for (int fifthLoad = 0; fifthLoad <= 3; fifthLoad ++) {
                                for (int sixthLoad = 0; sixthLoad <= 3; sixthLoad ++) {
                                    cipherTest.vumarkGlyph(load[firstLoad],vumarkColumn);
                                    cipherTest.search(load[secondLoad]);
                                    cipherTest.search(load[thirdLoad]);
                                    cipherTest.search(load[fourthLoad]);
                                    cipherTest.search(load[fifthLoad]);
                                    cipherTest.search(load[sixthLoad]);

                                    System.out.println(cipherTest.glyphColumn(vumarkColumn)
                                            +", "+cipherTest.glyphColors(load[firstLoad])
                                            +", "+cipherTest.glyphColors(load[secondLoad])
                                            +", "+cipherTest.glyphColors(load[thirdLoad])
                                            +", "+cipherTest.glyphColors(load[fourthLoad])
                                            +", "+cipherTest.glyphColors(load[fifthLoad])
                                            +", "+cipherTest.glyphColors(load[sixthLoad])
                                            +", "+cipherTest.checkCipherValid()
                                            +", "+cipherTest.cipherNumber(cipherTest.cipherChosen));

                                    totalCount += 1;

                                    if (cipherTest.checkCipherValid()) {
                                        cipherCount += 1;
                                    }
                                    Ciphers.reset();
                                }
                            }
                        }
                    }
                }
            }
        }

        System.out.println("Total Count: "+totalCount);
        System.out.println("Compatible Count: "+cipherCount);
    }

    private static int[][][] load = new int[][][] {
            {
                    {0}, //fake bottom glyph to simulate solid surface below
                    {1}, //1st glyph intaked (bottom when placing)
                    {1}  //2nd glyph (top when placing)
            },
            {
                    {0}, //fake bottom glyph to simulate solid surface below
                    {1}, //1st glyph intaked (bottom when placing)
                    {2}  //2nd glyph (top when placing)
            },
            {
                    {0}, //fake bottom glyph to simulate solid surface below
                    {2}, //1st glyph intaked (bottom when placing)
                    {1}  //2nd glyph (top when placing)
            },
            {
                    {0}, //fake bottom glyph to simulate solid surface below
                    {2}, //1st glyph intaked (bottom when placing)
                    {2}  //2nd glyph (top when placing)
            }
    };

}