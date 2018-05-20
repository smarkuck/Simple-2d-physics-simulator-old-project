package Physics2D;
/**
 * Klasa odpowiedzialna za obliczenia wykonywane na punktach w przestrzeni
 *
 * @version 1.0
 * @since   2018-05-19
 */
public class Vector2 {

    public double x = 0;
    public double y = 0;

    //-------------------------------------------------------------------------------
    /**
     * Domyślny konstruktor klasy Vector2 ustawia obie współrzędne x i y na 0
     */
    public Vector2() {
        x = 0;
        y = 0;
    }

    //-------------------------------------------------------------------------------
    /**
     * Konstruktor klasy Vector2
     * @param x x
     * @param  y y
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //static functions return new vector
    //non-static functions return modified vector
    //-------------------------------------------------------------------------------
    /**
     * Funkcja statyczna, zwraca całkiem nowy wektor stworzony z sumowanie dwóch przesłanych obiektów
     * @param a wektor pierwszy
     * @param b wektor drugi
     * @return  nowy wektor - suma przesłanych w parametrze
     */
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Funkcja modyfikuje istniejący obiekt klasy wektor, dodawanie wektorów, wynik wpisany w nasz obiekt
     * @param a wektor którego współrzędne dodamy do współrzędnych naszego obiektu
     * @return referencja do naszego obiektu zmodyfikowanego
     */
    public Vector2 add(Vector2 a) {
        this.x += a.x;
        this.y += a.y;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Funkcja statyczna, zwraca całkiem nowy wektor wynik odejmowanie dwóch przesłanych wektorów
     * @param a wektor pierwszy
     * @param b wektor drugi
     * @return nowy wektor powstały po operacji odejmowania dwóch przesłanych w parametrze wektorów
     */
    public static Vector2 subtract(Vector2 a, Vector2 b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Funkcja modyfikuje istniejący obiekt klasy wektor, odejmowanie wektorów, wynik wpisany w nasz obiekt
     * @param a wektor którego współrzędne odejmiemy od współrzędnych naszego obiektu
     * @return referencja do naszego obiektu zmodyfikowanego
     */
    public Vector2 subtract(Vector2 a) {
        this.x -= a.x;
        this.y -= a.y;
        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Funkcja statyczna, zwraca całkiem nowy wektor wynik mnożenia przesłanego skalara przez przesłany wektor
     * @param a skalra
     * @param b wektor
     * @return nowy wektor powstały po operacji mnożenia
     */
    public static Vector2 multiply(double a, Vector2 b)
    {
        return new Vector2(a*b.x,a*b.y);
    }

    //-------------------------------------------------------------------------------

    /**
     * Funkcja statyczna, zwraca całkiem nowy wektor wynik mnożenia przesłanego skalara przez przesłany wektor
     * @param a wektor
     * @param b skalar
     * @return nowy wektor powstały po operacji mnożenia
     */

    public static Vector2 multiply(Vector2 a, double b)
    {
        return new Vector2(b*a.x,b*a.y);
    }

    //-------------------------------------------------------------------------------

    /**
     * Funkcja modyfikuje istniejący obiekt klasy wektor, mnożenie wektora przez skalar, wynik wpisany w nasz obiekt
     * @param a skalar przez którego mnożymy współrzędne naszego obiektu
     * @return referencja do naszego obiektu zmodyfikowanego
     */

    public Vector2 multiply(double a) {
        this.x *= a;
        this.y *= a;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Funkcja statyczna, zwraca całkiem nowy wektor wynik dzielenia przesłanego wektora przez przesłany skalar
     * @param a wektor
     * @param b skalar
     * @return nowy wektor powstały po operacji dzielenia
     */
    public static Vector2 divide(Vector2 a, double b) {
        return new Vector2(a.x/b, a.y/b);
    }

    //-------------------------------------------------------------------------------
    /**
     * Funkcja modyfikuje istniejący obiekt klasy wektor, dzielenie wektora przez skalar, wynik wpisany w nasz obiekt
     * @param a skalar przez którego dzielimy nasze współrzędne
     * @return referencja do naszego obiektu zmodyfikowanego
     */

    public Vector2 divide(double a) {
        this.x /= a;
        this.y /= a;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwraca iloczyn skalarny wektorów, uwzględnia część drugiego wektora skierowaną w stronę pierwszego
     * @return Vector2 nowy wektor
     */
    public double DotProduct(Vector2 a) {
        return this.x*a.x + this.y*a.y;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwraca iloczyn skalarny wektorów, uwzględnia część drugiego wektora skierowaną w stronę przeciwna do naszego
     * @return Vector2 nowy wektor
     */
    public double PerpDotProduct(Vector2 a) {
        return this.x*a.y - this.y*a.x;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwraca wektor prostopadły do naszego wektora
     * @return Vector2 nowy wektor prostopadłu do naszego
     */
    public Vector2 GetPerpendicular() {
        return new Vector2(-this.y, this.x);
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwraca długość wektora
     * @return double długość wektora
     */
    public double GetLength()
    {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwraca wektor normalny do naszego wektora, taki sam ale o długości jeden
     * @return Vector2 nowy wektor będący wektorem normalnym dla naszego wektora
     */
    public Vector2 GetNormal() {
        double OneOverLength = 1/GetLength();
        return new Vector2(OneOverLength*this.x, OneOverLength*this.y);
    }
}
