package Physics2D;

public class Force {

    public Vector2 getPosition() {
        return new Vector2(Position.x, Position.y);
    }

    //-------------------------------------------------------------------------------

    public Force setPosition(Vector2 position) {
        Position.x = position.x;
        Position.y = position.y;

        return this;
    }

    //-------------------------------------------------------------------------------

    public boolean isConstant() {
        return isConstant;
    }

    //-------------------------------------------------------------------------------

    public Force IsConstant(boolean option) {
        isConstant = option;
        return this;
    }

    //-------------------------------------------------------------------------------

    public double getValue() {
        return value;
    }

    //-------------------------------------------------------------------------------

    public Force setValue(double value) {
        this.value = value;
        return this;
    }

    //-------------------------------------------------------------------------------

    public double getRange() {
        return range;
    }

    //-------------------------------------------------------------------------------

    public Force setRange(double range) {
        this.range = range;
        return this;
    }

    //===============================================================================
    //===============================================================================

    Vector2 Position;
    boolean isConstant;
    double value;
    double range;
}