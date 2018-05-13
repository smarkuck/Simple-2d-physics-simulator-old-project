import com.sun.org.glassfish.external.statistics.impl.BoundaryStatisticImpl;

public class RigidBody {

    double Width, Height;
    double OneOverMass, OneOverCMMomentOfInertia;
    double CoefficientOfRestitution;

    static final int NumberOfConfigurations = 2;

    Configuration[] configurations = new Configuration[NumberOfConfigurations];

    //-------------------------------------------------------------------------------

    public RigidBody(double Density, double Width, double Height, double CoefficientOfRestitution) {
        final double Mass = Density * Width * Height;

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
