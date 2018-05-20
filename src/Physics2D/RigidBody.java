package Physics2D;

/**
 * Klasa odpowiadająca za wlaściwości ciała położonego w przestrzeni
 *
 * @version 1.0
 * @since   2018-05-19
 */
public class RigidBody {

    double Width, Height;
    double OneOverMass, OneOverCMMomentOfInertia;
    double CoefficientOfRestitution;

    boolean isStatic;

    static final int NumberOfConfigurations = 2;

    // są dwie konfiguracje aktualna i docelowa w drugiej wszystko jest przeliczone jakby wyglądało i jest sprawdzenie czy nic się nie
    //zderza jak się zderza to czas się trochę cofa żeby był przed kolizją i znowu przelicza docelową konfigurację
    Configuration[] configurations = new Configuration[NumberOfConfigurations];

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca szerokość ciała
     * @return double szerokość ciała
     */
    public double getWidth() {
        return Width;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca szerokość ciała
     * @param width szerokość ciała
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setWidth(double width) {
        if(width <= 0)
            this.Width = 0.1;
        else
            this.Width = width;

        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca wysokość ciała
     * @return double wysokość ciała
     */
    public double getHeight() {
        return Height;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca wysokość ciała
     * @param height wysokość ciała
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setHeight(double height) {
        if(height <= 0)
            this.Height = 0.1;
        else
            this.Height = height;

        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca masę ciała
     * @return double masa ciała
     */
    public double getMass() {
        return 1/OneOverMass;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca szerokość ciała
     * @param mass masa ciała
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setMass(double mass) {
        if(mass <= 0)
            OneOverMass = 10;
        else
            OneOverMass = 1/mass;

        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca współczynnik sprężystości
     * @return double współczynnik sprężystości
     */
    public double getCoefficientOfRestitution() {
        return CoefficientOfRestitution;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca współczynnik sprężystości
     * @param coefficient współczynnik sprężystości
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setCoefficientOfRestitution(double coefficient) {
        if(coefficient < 0)
            CoefficientOfRestitution = 0;
        else
            CoefficientOfRestitution = coefficient;

        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda sprawdzająca czy obiekt jest statyczny - nieruchomy
     * @return boolean jeśli jest nieruchomy zwraca true
     */
    public boolean isStatic() {
        return isStatic;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca czy obiekt może się poruszać
     * @param option true - obiekt ruchomy false - obiekt nieruchomy
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody isStatic(boolean option) {
        isStatic = option;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca środek ciężkości
     * @return Vector2 środek ciężkości
     */
    public Vector2 getCMPosition() {
        return new Vector2(configurations[0].CMPosition.x, configurations[0].CMPosition.y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca środek ciężkości
     * @param position Vector2 nowe położenie środka ciężkości
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setCMPosition(Vector2 position) {
        configurations[0].CMPosition.x = position.x;
        configurations[0].CMPosition.y = position.y;

        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca prędkość środka ciężkości obiektu
     * @return Vector2 zwraca wektor prędości
     */
    public Vector2 getCMVelocity() {
        return new Vector2(configurations[0].CMVelocity.x, configurations[0].CMVelocity.y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca prędkośc środka ciężkości
     * @param velocity wektor prędkości
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setCMVelocity(Vector2 velocity) {
        configurations[0].CMVelocity.x = velocity.x;
        configurations[0].CMVelocity.y = velocity.y;

        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca wartość obrotu ciała w stopniach w stronę przeciwną do ruchu wskazówek zegara
     * @return double wartość w stopniach, liczona w stronę przeciwną do ruchu wskazówek zegara
     */
    public double getOrientation() {
        return configurations[0].Orientation;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca wartość obrotu ciała w stopniach
     * @param orientation wartość w stopniach, liczona w stronę przeciwną do ruchu wskazówek zegara
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setOrientation(double orientation) {
        configurations[0].Orientation = orientation;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca prędkość kątową obiektu
     * @return double prędkość kątowa obiektu
     */
    public double getAngularVelocity() {
        return configurations[0].AngularVelocity;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca prędkość kątową obiektu
     * @param velocity prędkość katowa obiektu
     * @return RigitBody obiekt RigitBody zwraca referencję do siebie samego
     */
    public RigidBody setAngularVelocity(double velocity) {
        configurations[0].AngularVelocity = velocity;
        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca wartość wybranego wierchołka obiektu
     * @param index wartość powinna być z zakresu <1;4> ponieważ obiekty mają 4 wierzchołki
     * @return Vector2 gdy wartość index prawidłowa w przypadku błędu - null
     */
    public Vector2 getVertex(int index) {

        if(index < 1 || index > 4)
            return null;
        --index;

        return new Vector2(configurations[0].Box.vertices[index].x, configurations[0].Box.vertices[index].y);
    }

    //===============================================================================
    //===============================================================================


    /**
     * Konstruktor klasy RigitBody
     * @param Density gęstość ciała
     * @param Width szerokość ciała
     * @param Height wysokość ciała
     * @param CoefficientOfRestitution współczynnik sprężystości
     * @param isStatic czy ma mieć możliwośc poruszania ( true - tak )
     */
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
