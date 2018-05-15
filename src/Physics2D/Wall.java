package Physics2D;

class Wall {

    //===============================================================================
    //===============================================================================

    // define wall by plane equation
    Vector2 Normal;		// inward pointing
    double c;					// ax + by + c = 0

    // points for drawing wall
    Vector2 StartPoint;
    Vector2 EndPoint;
}
