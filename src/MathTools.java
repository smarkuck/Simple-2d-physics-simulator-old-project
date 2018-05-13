public class MathTools {
    public static final double REAL_MAX = Float.MAX_VALUE;
    public static final double REAL_MIN = Float.MIN_VALUE;

    public static final double PI = 3.14159265358979323846;
    public static final double Epsilon = 0.00001;

    //-------------------------------------------------------------------------------

    public static double RadiansFrom(double Degrees)
    {
        return (Degrees * PI) / 180.0;
    }

    //-------------------------------------------------------------------------------

    public static double DegreesFrom(double Radians)
    {
        return (Radians * 180) / PI;
    }
}
