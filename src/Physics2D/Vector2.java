package Physics2D;

public class Vector2 {

    public double x = 0;
    public double y = 0;

    //-------------------------------------------------------------------------------
    
    public Vector2() {
        x = 0;
        y = 0;
    }

    //-------------------------------------------------------------------------------
    
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //static functions return new vector
    //non-static functions return modified vector
    //-------------------------------------------------------------------------------
    
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 add(Vector2 a) {
        this.x += a.x;
        this.y += a.y;
        return this;
    }

    //-------------------------------------------------------------------------------
    
    public static Vector2 subtract(Vector2 a, Vector2 b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 subtract(Vector2 a) {
        this.x -= a.x;
        this.y -= a.y;
        return this;
    }

    //-------------------------------------------------------------------------------
    
    public static Vector2 multiply(double a, Vector2 b)
    {
        return new Vector2(a*b.x,a*b.y);
    }

    //-------------------------------------------------------------------------------
    
    public static Vector2 multiply(Vector2 a, double b)
    {
        return new Vector2(b*a.x,b*a.y);
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 multiply(double a) {
        this.x *= a;
        this.y *= a;
        return this;
    }

    //-------------------------------------------------------------------------------
    
    public static Vector2 divide(Vector2 a, double b) {
        return new Vector2(a.x/b, a.y/b);
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 divide(double a) {
        this.x /= a;
        this.y /= a;
        return this;
    }

    //-------------------------------------------------------------------------------
    
    public double DotProduct(Vector2 a) {
        return this.x*a.x + this.y*a.y;
    }

    //-------------------------------------------------------------------------------
    
    public double PerpDotProduct(Vector2 a) {
        return this.x*a.y - this.y*a.x;
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 GetPerpendicular() {
        return new Vector2(-this.y, this.x);
    }

    //-------------------------------------------------------------------------------
    
    public double GetLength()
    {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }

    //-------------------------------------------------------------------------------
    
    public Vector2 GetNormal() {
        double OneOverLength = 1/GetLength();
        return new Vector2(OneOverLength*this.x, OneOverLength*this.y);
    }
}
