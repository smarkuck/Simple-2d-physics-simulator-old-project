package Physics2D;

public class RigidBody {

    public double getWidth() {
        return Width;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setWidth(double width) {
        if(width <= 0)
            this.Width = 0.1;
        else
            this.Width = width;

        return this;
    }

    //-------------------------------------------------------------------------------

    public double getHeight() {
        return Height;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setHeight(double height) {
        if(height <= 0)
            this.Height = 0.1;
        else
            this.Height = height;

        return this;
    }

    //-------------------------------------------------------------------------------

    public double getMass() {
        return 1/OneOverMass;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setMass(double mass) {
        if(mass <= 0)
            OneOverMass = 10;
        else
            OneOverMass = 1/mass;

        return this;
    }

    //-------------------------------------------------------------------------------

    public double getCoefficientOfRestitution() {
        return CoefficientOfRestitution;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setCoefficientOfRestitution(double coefficient) {
        if(coefficient < 0)
            CoefficientOfRestitution = 0;
        else
            CoefficientOfRestitution = coefficient;

        return this;
    }

    //-------------------------------------------------------------------------------

    public boolean isStatic() {
        return isStatic;
    }

    //-------------------------------------------------------------------------------

    public RigidBody isStatic(boolean option) {
        isStatic = option;
        return this;
    }

    //-------------------------------------------------------------------------------

    public Vector2 getCMPosition() {
        return new Vector2(configurations[0].CMPosition.x, configurations[0].CMPosition.y);
    }

    //-------------------------------------------------------------------------------

    public RigidBody setCMPosition(Vector2 position) {
        configurations[0].CMPosition.x = position.x;
        configurations[0].CMPosition.y = position.y;

        return this;
    }

    //-------------------------------------------------------------------------------

    public Vector2 getCMVelocity() {
        return new Vector2(configurations[0].CMVelocity.x, configurations[0].CMVelocity.y);
    }

    //-------------------------------------------------------------------------------

    public RigidBody setCMVelocity(Vector2 velocity) {
        configurations[0].CMVelocity.x = velocity.x;
        configurations[0].CMVelocity.y = velocity.y;

        return this;
    }

    //-------------------------------------------------------------------------------

    public double getOrientation() {
        return configurations[0].Orientation;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setOrientation(double orientation) {
        configurations[0].Orientation = orientation;
        return this;
    }

    //-------------------------------------------------------------------------------

    public double getAngularVelocity() {
        return configurations[0].AngularVelocity;
    }

    //-------------------------------------------------------------------------------

    public RigidBody setAngularVelocity(double velocity) {
        configurations[0].AngularVelocity = velocity;
        return this;
    }

    //-------------------------------------------------------------------------------

    public Vector2 getVertex(int index) {

        if(index < 1 || index > 4)
            return null;
        --index;

        return new Vector2(configurations[0].Box.vertices[index].x, configurations[0].Box.vertices[index].y);
    }

    //===============================================================================
    //===============================================================================

    double Width, Height;
    double OneOverMass, OneOverCMMomentOfInertia;
    double CoefficientOfRestitution;

    boolean isStatic;

    static final int NumberOfConfigurations = 2;

    Configuration[] configurations = new Configuration[NumberOfConfigurations];

    //-------------------------------------------------------------------------------

    RigidBody(double Density, double Width, double Height, double CoefficientOfRestitution, boolean isStatic) {
        final double Mass = Density * Width * Height;

        this.isStatic = isStatic;

        this.CoefficientOfRestitution = CoefficientOfRestitution;

        this.Width = Width;
        this.Height = Height;

        this.OneOverMass = 1 / Mass;

        // integrate over the body to find the moment of inertia

        this.OneOverCMMomentOfInertia = 1 / ((Mass / 12.0) *
                (Width * Width + Height * Height));

        this.configurations[0] = new Configuration();
        this.configurations[1] = new Configuration();
    }

    //-------------------------------------------------------------------------------

    class Configuration
    {
        Vector2 CMPosition = new Vector2();
        double Orientation = 0;

        Vector2 CMVelocity = new Vector2();
        double AngularVelocity = 0;

        Vector2 CMForce = new Vector2();
        double Torque = 0;

        BoundingBox Box = new BoundingBox();

        class BoundingBox
        {
            BoundingBox() {
                for(int i = 0; i < 4; i++) {
                    vertices[i] = new Vector2();
                }
            }

            Vector2[] vertices = new Vector2[4];
        }

    }
}
