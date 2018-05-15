package Physics2D;

import java.util.Vector;

public class Spring {

    public boolean attachedToWall() {
        return toWall;
    }

    //-------------------------------------------------------------------------------

    public Spring setWallAttachment(Vector2 position) {
        toWall = true;
        pos = new Vector2(position.x, position.y);

        return this;

    }

    //-------------------------------------------------------------------------------

    public boolean setBody1Attachment(RigidBody body, int vertex) {
        if(!setVertex1(vertex))
            return false;

        toWall = false;
        body1 = body;

        return true;
    }

    //-------------------------------------------------------------------------------

    public boolean setBody2Attachment(RigidBody body, int vertex) {
        if(!setVertex2(vertex))
            return false;

        body2 = body;

        return true;
    }

    //-------------------------------------------------------------------------------

    public Vector2 getAttachment1() {
        if(toWall)
            return new Vector2(pos.x, pos.y);
        else
            return new Vector2(body1.configurations[0].Box.vertices[vertex1].x,
                                body1.configurations[0].Box.vertices[vertex1].y);
    }

    //-------------------------------------------------------------------------------

    public Vector2 getAttachment2() {
        return new Vector2(body2.configurations[0].Box.vertices[vertex2].x,
                body2.configurations[0].Box.vertices[vertex2].y);
    }

    //-------------------------------------------------------------------------------

    public double getHooke() {
        return Hooke;
    }

    //-------------------------------------------------------------------------------

    public Spring setHooke(double Hooke) {
        if(Hooke <= 0)
            this.Hooke = 0.1;
        else
            this.Hooke = Hooke;

        return this;
    }

    //-------------------------------------------------------------------------------

    public double getDamping() {
        return damping;
    }

    //-------------------------------------------------------------------------------

    public Spring setDamping(double damping) {
        if(damping < 0)
            this.damping = 0;
        else
            this.damping = damping;

        return this;
    }

    //-------------------------------------------------------------------------------

    public boolean setVertex1(int vertex) {
        if(vertex < 1 || vertex > 4 || toWall == true)
            return false;

        --vertex;

        vertex1 = vertex;

        return true;
    }

    //-------------------------------------------------------------------------------

    public boolean setVertex2(int vertex) {
        if(vertex < 1 || vertex > 4)
            return false;

        --vertex;

        vertex2 = vertex;

        return true;
    }

    //-------------------------------------------------------------------------------

    public int getVertex1Index() {
        if(toWall == true)
            return 0;

        return vertex1;
    }

    //-------------------------------------------------------------------------------

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