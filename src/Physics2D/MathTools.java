package Physics2D;
/**
 * Klasa przechowująca stałe matematyczne oraz dokonująca przeliczeń kątów
 *
 * @version 1.0
 * @since   2018-05-19
 */
public class MathTools {

    /**
     * Metoda statyczna przeliczająca stopnie na radiany
     * @param Degrees wartość kąta w stopniach
     * @return double zwraca wartość kąta w radianach
     */
    public static double RadiansFrom(double Degrees)
    {
        return (Degrees * PI) / 180.0;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda statyczna przeliczająca radiany na stopnie
     * @param Radians wartość kąta w radianach
     * @return double zwraca wartość kąta w stopniach
     */
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
