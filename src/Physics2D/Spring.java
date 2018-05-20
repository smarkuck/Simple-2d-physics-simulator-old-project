package Physics2D;

import java.util.Vector;
/**
 * Klasa odpowiedzialna za "sprężyny" które można przyczepiać do punktów i obiektów
 *
 * @version 1.0
 * @since   2018-05-19
 */
public class Spring {

    /**
     * Metoda sprawdzająca czy sprężyna jest przyczepiona do punktu
     * @return boolean - gdy tak to true
     */
    public boolean attachedToWall() {
        return toWall;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda przyczepiająca spręzynę do wybranego punktu
     * @param position punkt do którego przyczepiamy sprężynę
     * @return zwraca spręzynę ktorą własnie przyczepiliśmy
     */
    public Spring setWallAttachment(Vector2 position) {
        toWall = true;
        pos = new Vector2(position.x, position.y);

        return this;

    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda przyczepiająca spręzynę do pierwszego wybranego obiektu
     * @param body obiekt do którego chcemy przyczepić naszę sprężynę
     * @param vertex numer wierzchołka ciała body do którego chcemy przyczepić sprężynę
     * @return zwraca boolean w zalezności od wyniku operacji, gdy się powiedzie to true, gdy nieprawidłowa wartość wierzchołka to false
     */
    public boolean setBody1Attachment(RigidBody body, int vertex) {
        if(!setVertex1(vertex))
            return false;

        toWall = false;
        body1 = body;

        return true;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda przyczepiająca spręzynę do drugiego wybranego obiektu
     * @param body obiekt do którego chcemy przyczepić naszę sprężynę
     * @param vertex numer wierzchołka ciała body do którego chcemy przyczepić sprężynę
     * @return zwraca boolean w zalezności od wyniku operacji, gdy się powiedzie to true, gdy nieprawidłowa wartość wierzchołka to false
     */
    public boolean setBody2Attachment(RigidBody body, int vertex) {
        if(!setVertex2(vertex))
            return false;

        body2 = body;

        return true;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca punkt do którego przyczepiona jest sprężyna,
     * @return jeśli jest przyczepiona do punktu zwraca jego współrzędne, jeśli do ciała to zwraca aktualne współrzędne wierzchołka pierwszego obiektu do którego jest przyczepiona
     */
    public Vector2 getAttachment1() {
        if(toWall)
            return new Vector2(pos.x, pos.y);
        else
            return new Vector2(body1.configurations[0].Box.vertices[vertex1].x,
                                body1.configurations[0].Box.vertices[vertex1].y);
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca współrzędne wierzchołka drugiego obiektu do którego przyczepiona jest sprężyna
     * @return zwraca aktualne współrzędne wierzchołka obiektu do którego jest przyczepiona
     */
    public Vector2 getAttachment2() {
        return new Vector2(body2.configurations[0].Box.vertices[vertex2].x,
                body2.configurations[0].Box.vertices[vertex2].y);
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda zwracająca współczynnik sprężystości sprężyny
     * @return double
     */
    public double getHooke() {
        return Hooke;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca współczynnik sprężystości sprężyny
     * @param Hooke współczynnik sprężystości, gdy jest mniejszy od 0 jest ustawiany na 0.1
     * @return zwraca referencję do sprężyny
     */
    public Spring setHooke(double Hooke) {
        if(Hooke <= 0)
            this.Hooke = 0.1;
        else
            this.Hooke = Hooke;

        return this;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca współczynnik wytłumienia spręzyny
     * @return double
     */
    public double getDamping() {
        return damping;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca współczynnik sprężystości sprężyny
     * @param Hooke współczynnik sprężystości, gdy jest mniejszy lub równy 0 jest ustawiany na 0
     * @return zwraca referencję do sprężyny
     */
    public Spring setDamping(double damping) {
        if(damping < 0)
            this.damping = 0;
        else
            this.damping = damping;

        return this;
    }

    //-------------------------------------------------------------------------------

    /**
     * Metoda ustawiająca wierzchołek pierwszego obiektu do którego jest chcemy przyczepić sprężynę
     * @param vertex numer wierzchołka z zakresu <1;4>
     * @return gdy uda się przyczepić sprężynę do wierzchołka zwraca true
     */
    public boolean setVertex1(int vertex) {
        if(vertex < 1 || vertex > 4 || toWall == true)
            return false;

        --vertex;

        vertex1 = vertex;

        return true;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda ustawiająca wierzchołek drugiego obiektu do którego jest chcemy przyczepić sprężynę
     * @param vertex numer wierzchołka z zakresu <1;4>
     * @return gdy uda się przyczepić sprężynę do wierzchołka zwraca true
     */
    public boolean setVertex2(int vertex) {
        if(vertex < 1 || vertex > 4)
            return false;

        --vertex;

        vertex2 = vertex;

        return true;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca numer wierzchołka pierwszego obiektu do którego przyczepiona jest sprężyna
     * @return gdy sprężyna jest przyczepiona do punktu wtedy zwraca 0, w innym przypadku nr wierzchołka
     */
    public int getVertex1Index() {
        if(toWall == true)
            return 0;

        return vertex1;
    }

    //-------------------------------------------------------------------------------
    /**
     * Metoda zwracająca numer wierzchołka drugiego obiektu do którego przyczepiona jest sprężyna
     * @return gdy sprężyna jest przyczepiona do punktu wtedy zwraca 0, w innym przypadku nr wierzchołka
     */
    public int getVertex2Index() {
        return vertex2;
    }

    //===============================================================================
    //===============================================================================

    double Hooke;
    double damping;
    Vector2 pos;
    RigidBody body1;
    RigidBody body2;
    int vertex1;
    int vertex2;
    boolean toWall;

    //-------------------------------------------------------------------------------
    /**
     * Konstruktor klasy Spring,
     * Tworzenie sprężyny przyczepionej do punktu i do obiektu
     * @param position punkt w którym chcemy umieścić sprężynę
     * @param body ciało do którego chcemy przyczepić sprężynę
     * @param vertex numer wierzchołka ciała body <1;4> do którego chcemy przyczepić sprężynę
     * @param hooke współczynnik sprężystości, gdy mniejszy od 0 wtedy ustawiamy go na 0.1
     * @param damping współczynnik wytłumienia gdy wartosc jest mniejsza od 0 wtedy automatycznie jest ustawiana na 0
     */
    Spring(Vector2 position, RigidBody body, int vertex, double hooke, double damping) {
        if(vertex < 1)
            vertex = 1;
        else if(vertex > 4)
            vertex = 4;

        if(hooke <= 0)
            hooke = 0.1;

        if(damping < 0)
            damping = 0;

        --vertex;

        pos = position;
        vertex2 = vertex;
        body2 = body;
        this.Hooke = hooke;
        this.damping = damping;
        this.toWall = true;
    }

    //-------------------------------------------------------------------------------
    /**
     * Konstruktor klasy Spring,
     * Tworzenie sprężyny przyczepionej do dwóch obiektów
     * @param body1 pierwszy obiekt do którego chcemy przyczepić sprężynę
     * @param vertex1 numer wierzchołka pierwszego obiektu body1 <1;4> do którego chcemy przyczepić sprężynę
     * @param body2 drugi obiekt do którego chcemy przyczepić sprężynę
     * @param vertex2 numer wierzchołka drugiego obiektu body1 <1;4> do którego chcemy przyczepić sprężynę
     * @param hooke współczynnik sprężystości, gdy mniejszy od 0 wtedy ustawiamy go na 0.1
     * @param damping współczynnik wytłumienia gdy wartosc jest mniejsza od 0 wtedy automatycznie jest ustawiana na 0
     */
    Spring(RigidBody body1, int vertex1, RigidBody body2, int vertex2, double hooke, double damping) {
        if(vertex1 < 1)
            vertex1 = 1;
        else if(vertex1 > 4)
            vertex1 = 4;

        if(vertex2 < 1)
            vertex2 = 1;
        else if(vertex2 > 4)
            vertex2 = 4;

        if(hooke <= 0)
            hooke = 0.1;

        if(damping < 0)
            damping = 0;

        --vertex1;
        --vertex2;

        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.body1 = body1;
        this.body2 = body2;
        this.Hooke = hooke;
        this.damping = damping;
        this.toWall = false;
    }
}