package Physics2D;

public class MathTools {

    public static double RadiansFrom(double Degrees)
    {
        return (Degrees * PI) / 180.0;
    }

    //-------------------------------------------------------------------------------

    public static double DegreesFrom(double Radians)
    {
        return (Radians * 180) / PI;
    }

    //===============================================================================
    //===============================================================================

    static final double REAL_MAX = Float.MAX_VALUE;
    static final double REAL_MIN = Float.MIN_VALUE;

    static final double PI = 3.14159265358979323846;
    static final double Epsilon = 0.00001;

    //-------------------------------------------------------------------------------
}
