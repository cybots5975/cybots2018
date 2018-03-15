package org.firstinspires.ftc.teamcode.test.multiGluph;

/**
 * https://codereview.stackexchange.com/questions/138442/pattern-searching-in-2d-grid
 */

public class GridSearcher {

    private static final int delta_i[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    private static final int delta_j[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

    private char[][] grid;

    public GridSearcher(char[][] grid) {
        this.grid = grid;
    }

    public boolean isAllowed(int row, int col){
        return (row >= 0 && row < grid.length && col >= 0 && col < grid[row].length);
    }

    public boolean searchGrid(int row, int col, String pat, boolean[][] visited, int pInd) {

        visited[row][col] = true;
        pInd++;
        if (pInd >= pat.length()) {
            return true;
        }

        for (int dir = 0; dir < delta_i.length; dir++) {
            int row_i = row + delta_i[dir];
            int col_j = col + delta_j[dir];
            if (!isAllowed(row_i, col_j) || visited[row_i][col_j]) {
                continue;
            }

            if (grid[row_i][col_j] == pat.charAt(pInd)) {
                if (searchGrid(row_i, col_j, pat, visited, pInd)) {
                    return true;
                }
            }
        }

        // backtracking visited
        visited[row][col] = false;

        return false;
    }

    public boolean find(String needle) {
        boolean[][] visited = new boolean[grid.length][grid[0].length];
        for (int i=0; i < grid.length; i++){
            for (int j=0; j < grid[0].length; j++) {
                if (grid[i][j] == needle.charAt(0) && searchGrid(i, j, needle, visited, 0)) {
                    return true;
                }
            }
        }

        return false;
    }

    public int getCount() {
        return grid.length * grid[0].length;
    }

    public static void main(String[] args) {
        char[][] grid
                = { { 'A', 'C', 'P', 'R', 'C' },
                { 'X', 'S', 'O', 'P', 'C' },
                { 'V', 'O', 'V', 'N', 'I' },
                { 'W', 'G', 'F', 'M', 'N' },
                { 'Q', 'A', 'T', 'I', 'T' } };
        GridSearcher searcher = new GridSearcher(grid);

        String [] needles = {"MICROSOFT", "ZZZ", "Oa"};
        for (String needle : needles) {
            if (searcher.find(needle)) {
                System.out.println("Present");
            } else {
                System.out.println("Not Present   " + searcher.getCount());
            }
        }

    }

}
