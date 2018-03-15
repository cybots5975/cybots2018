package org.firstinspires.ftc.teamcode.test.multiGluph;

/*
 * The Knuth-Morris-Pratt Algorithm for Pattern Matching
 * https://store.fmi.uni-sofia.bg/fmi/logic/vboutchkova/sources/KMPMatch.java
 *
 * http://en.wikipedia.org/wiki/Knuth–Morris–Pratt_algorithm
 */
public class PatternSearch1 {

    private String string;
    private String pattern;
    private int[] failure;
    private int matchPoint;

    public PatternSearch1(String string, String pattern) {
        this.string = string;
        this.pattern = pattern;
        failure = new int[pattern.length()];
        computeFailure();
    }

    public int getMatchPoint() {
        return matchPoint;
    }


    public boolean match() {
        // Tries to find an occurence of the pattern in the string

        int j = 0;
        if (string.length() == 0) return false;

        for (int i = 0; i < string.length(); i++) {
            while (j > 0 && pattern.charAt(j) != string.charAt(i)) {
                j = failure[j - 1];
            }
            if (pattern.charAt(j) == string.charAt(i)) { j++; }
            if (j == pattern.length()) {
                matchPoint = i - pattern.length() + 1;
                return true;
            }
        }
        return false;
    }

    public boolean match1() {

        int i = 0;
        int j = 0;
        if (string.length() == 0) return false;

        while (i + pattern.length() - j <= string.length()) {
            if (j >= pattern.length()) {
                matchPoint = i - pattern.length();
                return true;
            }
            if (string.charAt(i) == pattern.charAt(j)) {
                i++;
                j++;
            } else {
                if (j > 0) { j = failure[j - 1]; }
                else { i++; }
            }
        }
        return false;
    }

    /**
     * Computes the failure function using a boot-strapping process,
     * where the pattern is matched against itself.
     */
    private void computeFailure() {

        int j = 0;
        for (int i = 1; i < pattern.length(); i++) {
            while (j > 0 && pattern.charAt(j) != pattern.charAt(i)) { j = failure[j - 1]; }
            if (pattern.charAt(j) == pattern.charAt(i)) { j++; }
            failure[i] = j;
        }
    }

}