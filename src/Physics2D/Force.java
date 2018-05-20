package Physics2D;
/**
 * Klasa odwzorowująca działanie sił punktowych
 *
 * @version 1.0
 * @since   2018-05-19
 */
public class Force {

    Vector2 Position;
    boolean isConstant;
    double value;
    double range;
    /**
     * Metoda zwracająca miejsce pochodzenia siły
     * @return Vector2 zwraca punkt w którym siła ma źródło
     */
    public Vector2 getPosition() {
        return new Vector2(Position.x, Position.y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca miejsce pochodzenia siły
     * @param position Vector2
     * @return Force zwraca siłę z ustawionym punktem przyłożenia
     */
    public Force setPosition(Vector2 position) {
        Position.x = position.x;
        Position.y = position.y;

        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda sprawdzająca czy siła jest stała
     * @return boolean zwraca prawdę gdy siła jest stała
     */
    public boolean isConstant() {
        return isConstant;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda w której ustawiamy czy siła ma być stała czy nie, gdy jest nie stała
     * to wtedy wraz z odległością od punktu źródłowego siły maleje
     * @param option boolean
     * @return Force zwraca referencję na samą siebie
     */
    public Force IsConstant(boolean option) {
        isConstant = option;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca wartość siły
     * @return double
     */
    public double getValue() {
        return value;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda w której ustawiamy wartość siły
     * @param value boolean
     * @return Force zwraca referencję na samą siebie
     */
    public Force setValue(double value) {
        this.value = value;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca zakres w jakim działa siła
     * @return double
     */
    public double getRange() {
        return range;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda w której ustawiamy czy siła ma być stała czy nie
     * @param range boolean
     * @return Force zwraca referencję na samą siebie
     */
    public Force setRange(double range) {
        this.range = range;
        return this;
    }

    //===============================================================================
    //===============================================================================

}