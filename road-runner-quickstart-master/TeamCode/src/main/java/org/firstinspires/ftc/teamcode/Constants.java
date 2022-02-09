package org.firstinspires.ftc.teamcode;

public class Constants {

    private static double[] xDist = new double[] {10, 8, 12}; //zone 1,2, 3
    private static double[] armDeg = new double[] {180, 380, 800}; //zone 1,2,3
    public static int[] lowCyanBounds = new int[] {106, 187, 50};
    public static int[] highCyanBounds = new int[] {118,225,255};
    public static int test = 0;

    public static double chooseX(int zone) {
        if(zone == 1) return xDist[0];
        else if(zone == 2) return xDist[1];
        else return xDist[2];
    }

    public static double chooseDeg(int zone) {
        if(zone == 1) return armDeg[0];
        else if(zone == 2) return armDeg[1];
        else return armDeg[2];
    }
}
