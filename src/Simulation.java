import javafx.geometry.Orientation;

import java.awt.*;
import java.util.ArrayList;

public class Simulation {

    double WorldWidth, WorldHeight;

    static final int NumberOfWalls = 5;
    Wall[] Walls = new Wall[NumberOfWalls];

    class Wall
    {
        // define wall by plane equation
        Vector2 Normal;		// inward pointing
        double c;					// ax + by + c = 0

        // points for drawing wall
        Vector2 StartPoint;
        Vector2 EndPoint;
    }

    int NumberOfBodies = 0;
    ArrayList<RigidBody> Bodies = new ArrayList<RigidBody>();

    boolean WorldSpringActive = false;		// spring goes from body 0: vertex 0 to origin
    double Kws = 30;			// Hooke's spring constant
    double Kwd = 5;			// damping constant
    Vector2 WorldSpringAnchor = new Vector2(0,0);

    boolean BodySpringActive = false;		// spring goes from body 0 to body 1
    double Kbs = 10;			// Hooke's spring constant
    double Kbd = 5;			// damping constant
    int Body0SpringVertexIndex = 2;
    int Body1SpringVertexIndex = 0;

    boolean GravityActive = true;
    Vector2 Gravity = new Vector2(0, -100);

    boolean DampingActive = false;
    double Kdl = 2.5;		// linear damping factor
    double Kda = 1400;

    int error = 0;

    enum CollisionState
    {
        Penetrating,
        Colliding,
        Clear
    }

    enum CollisionType {
        BoxWall,
        BoxBox
    }

    class Collision {
        Vector2 CollisionNormal;
        int CollidingBodyIndex;
        int CollidingBodyIndex2;
        int CollidingCornerIndex;

        CollisionState collisionState;
        CollisionType collisionType;
    }

    CollisionState collisionState;

    ArrayList<Collision> collisions = new ArrayList<Collision>();

    int SourceConfigurationIndex;
    int TargetConfigurationIndex;

    //-------------------------------------------------------------------------------

    public void addRigidBody(double Density, double Width, double Height, double CoefficientOfRestitution) {
        Bodies.add(new RigidBody(Density, Width, Height, CoefficientOfRestitution));
        ++NumberOfBodies;
    }

    //-------------------------------------------------------------------------------

    public Simulation(double WorldWidth, double WorldHeight) {

        this.WorldWidth = WorldWidth;
        this.WorldHeight = WorldHeight;
        SourceConfigurationIndex = 0;
        TargetConfigurationIndex = 1;

        for(int i = 0; i < NumberOfWalls; i++) {
            Walls[i] = new Wall();
        }

        // initialize walls
        Walls[0].Normal = new Vector2(0, -1);
        Walls[0].c = WorldHeight/2 - 3;

        Walls[1].Normal = new Vector2(0, 1);
        Walls[1].c = WorldHeight/2 - 3;

        Walls[2].Normal = new Vector2(-1 ,0);
        Walls[2].c = WorldWidth/2 - 3;

        Walls[3].Normal = new Vector2(1 ,0);
        Walls[3].c = WorldWidth/2 - 3;

        Walls[4].Normal = new Vector2(0.5, 1).GetNormal();
        Walls[4].c = WorldWidth/2;

        // generate the wall lines
        for(int Counter = 0; Counter < NumberOfWalls; Counter++)
        {
            Wall wall = Walls[Counter];

            // make a big line in the direction of the wall

            Vector2 PointOnWall = Vector2.multiply(-wall.c, wall.Normal);
            Vector2 v = wall.Normal.GetPerpendicular();
            double t0 = -MathTools.REAL_MAX;
            double t1 = MathTools.REAL_MAX;

            // now clip the line to the walls

            for(int WallIndex = 0; WallIndex < NumberOfWalls; WallIndex++)
            {
                if(WallIndex != Counter)
                {
                    Wall clipWall = Walls[WallIndex];

                    double Denominator = v.DotProduct(clipWall.Normal);

                    if(Math.abs(Denominator) > MathTools.Epsilon)
                    {
                        // not coplanar

                        double t = - (clipWall.c +
                                PointOnWall.DotProduct(clipWall.Normal)) /
                                Denominator;

                        if(Denominator > 0)
                        {
                            // the clip wall's clipping the t0 side of line
                            if(t > t0)
                            {
                                t0 = t;
                            }
                        }
                        else
                        {
                            // it's clipping the t1 side
                            if(t < t1)
                            {
                                t1 = t;
                            }
                        }
                    }
                }
            }

            // make sure we got clipped
            //assert((t0 != -REAL_MAX) && (t1 != REAL_MAX));
            // but not completely clipped
            //assert(t0 < t1);

            wall.StartPoint = Vector2.add(PointOnWall, Vector2.multiply(t0,v));
            wall.EndPoint = Vector2.add(PointOnWall, Vector2.multiply(t1,v));
        }

        // calculate initial box positions
        CalculateVertices(0);
    }

    //-------------------------------------------------------------------------------

    public void Simulate( double DeltaTime ) {

        double CurrentTime = 0;
        double TargetTime = DeltaTime;

        while(CurrentTime < DeltaTime)
        {
            ComputeForces(SourceConfigurationIndex);

            Integrate(TargetTime-CurrentTime);

            CalculateVertices(TargetConfigurationIndex);

            CheckForCollisions(TargetConfigurationIndex);

            if(collisionState == CollisionState.Penetrating)
            {
                // we simulated too far, so subdivide time and try again
                TargetTime = (CurrentTime + TargetTime) / 2.0;

                // blow up if we aren't moving forward each step, which is
                // probably caused by interpenetration at the frame start

                //assert(fabs(TargetTime - CurrentTime) > Epsilon);
                if(Math.abs(TargetTime - CurrentTime) < 0.0001) {
                    ++error;
                    double move = error / 100.;
                    //System.out.println("time < epsilon error: " + error + " move: " + move);
                    for(int i = 0; i < collisions.size(); i++) {
                        Collision state = collisions.get(i);
                        if(state.collisionState != CollisionState.Penetrating)
                            continue;

                        if(state.collisionType == CollisionType.BoxBox) {

                            //System.out.println("time crash");
                            RigidBody.Configuration conf = Bodies.get(state.CollidingBodyIndex).configurations[SourceConfigurationIndex];
                            RigidBody.Configuration conf2 = Bodies.get(state.CollidingBodyIndex2).configurations[SourceConfigurationIndex];

                            if (conf.CMPosition.x > conf2.CMPosition.x) {
                                conf.CMPosition.x += move;
                                conf2.CMPosition.x -= move;
                            } else {
                                conf.CMPosition.x -= move;
                                conf2.CMPosition.x += move;
                            }

                            if (conf.CMPosition.y > conf2.CMPosition.y) {
                                conf.CMPosition.y += move;
                                conf2.CMPosition.y -= move;
                            } else {
                                conf.CMPosition.y -= move;
                                conf2.CMPosition.y += move;
                            }
                        }
                        else {
                            RigidBody.Configuration conf = Bodies.get(state.CollidingBodyIndex).configurations[SourceConfigurationIndex];

                            conf.CMPosition.add(Vector2.multiply(move,state.CollisionNormal));
                            //conf.CMPosition.add(CollisionNormal);
                        }
                    }
                }
                collisions.clear();
            }
            else
            {
                error = 0;
                // either colliding or clear

                if(collisionState == CollisionState.Colliding)
                {
                    // @todo handle multiple simultaneous collisions

                    int Counter = 0;
                    do
                    {
                        ResolveCollisions(TargetConfigurationIndex);
                        collisions.clear();
                        Counter++;
                    } while((CheckForCollisions(TargetConfigurationIndex) ==
                            CollisionState.Colliding) && (Counter < 100));

                    //assert(Counter < 100);
                }

                // we made a successful step, so swap configurations
                // to "save" the data for the next step

                CurrentTime = TargetTime;
                TargetTime = DeltaTime;

                SourceConfigurationIndex = (SourceConfigurationIndex == 1) ? 0 : 1;
                TargetConfigurationIndex = (TargetConfigurationIndex == 1) ? 0 : 1;
            }
        }
    }

    //-------------------------------------------------------------------------------

    private void ComputeForces(int ConfigurationIndex) {
        int Counter;

        for(Counter = 0; Counter < NumberOfBodies; Counter++)
        {
            RigidBody Body = Bodies.get(Counter);
            RigidBody.Configuration Configuration =
                    Body.configurations[ConfigurationIndex];

            // clear forces

            Configuration.Torque = 0;
            Configuration.CMForce = new Vector2(0 ,0);

            if(GravityActive)
            {
                Configuration.CMForce.add(Vector2.divide(Gravity,Body.OneOverMass));
            }

            if(DampingActive)
            {
                Configuration.CMForce.add(Vector2.multiply(-Kdl,Configuration.CMVelocity));
                Configuration.Torque += -Kda * Configuration.AngularVelocity;
            }
        }

        if(BodySpringActive)
        {
            RigidBody Body0 = Bodies.get(0);
            RigidBody.Configuration Configuration0 =
                    Body0.configurations[ConfigurationIndex];
            RigidBody.Configuration.BoundingBox Box0 =
                Configuration0.Box;
            Vector2 Position0 = Box0.vertices[Body0SpringVertexIndex];
            Vector2 U0 = Vector2.subtract(Position0, Configuration0.CMPosition);
            Vector2 VU0 = Vector2.add(Configuration0.CMVelocity,
                    Vector2.multiply(Configuration0.AngularVelocity,U0.GetPerpendicular()));

            RigidBody Body1 = Bodies.get(1);
            RigidBody.Configuration Configuration1 =
                    Body1.configurations[ConfigurationIndex];
            RigidBody.Configuration.BoundingBox Box1 =
                Configuration1.Box;
            Vector2 Position1 = Box1.vertices[Body1SpringVertexIndex];
            Vector2 U1 = Vector2.subtract(Position1,Configuration1.CMPosition);
            Vector2 VU1 = Vector2.add(Configuration1.CMVelocity,
                    Vector2.multiply(Configuration1.AngularVelocity, U1.GetPerpendicular()));

            // spring goes from 0 to 1

            Vector2 SpringVector = Vector2.subtract(Position1, Position0);
            Vector2 Spring = Vector2.multiply(-Kbs,SpringVector);

            Vector2 RelativeVelocity = Vector2.subtract(VU1, VU0);
            // project velocity onto spring to get damping vector
            // this is basically a Gram-Schmidt projection
            Vector2 DampingForce =
                    Vector2.multiply(-Kbd * (RelativeVelocity.DotProduct(SpringVector)/
                    SpringVector.DotProduct(SpringVector)), SpringVector);

            Spring.add(DampingForce);

            Configuration0.CMForce.subtract(Spring);
            Configuration0.Torque -= U0.PerpDotProduct(Spring);

            Configuration1.CMForce.add(Spring);
            Configuration1.Torque += U1.PerpDotProduct(Spring);
        }

        if(WorldSpringActive)
        {
            // apply spring to body 0's vertex 0 to anchor

            RigidBody Body = Bodies.get(0);
            RigidBody.Configuration Configuration =
                    Body.configurations[ConfigurationIndex];
            RigidBody.Configuration.BoundingBox Box =
                Configuration.Box;

            Vector2 Position = Box.vertices[0];

            Vector2 U = Vector2.subtract(Position, Configuration.CMPosition);
            Vector2 VU = Vector2.add(Configuration.CMVelocity,
                    Vector2.multiply(Configuration.AngularVelocity, U.GetPerpendicular()));

            Vector2 Spring = Vector2.multiply(-Kws, Vector2.subtract(Position, WorldSpringAnchor));
            // project velocity onto spring to get damping vector
            // this is basically a Gram-Schmidt projection
            Vector2 DampingForce =
                    Vector2.multiply(-Kwd * (VU.DotProduct(Spring)/Spring.DotProduct(Spring)), Spring);

            Spring.add(DampingForce);

            Configuration.CMForce.add(Spring);
            Configuration.Torque += U.PerpDotProduct(Spring);
        }
    }

    //-------------------------------------------------------------------------------

    private void Integrate(double DeltaTime) {
        int Counter;

        for(Counter = 0; Counter < NumberOfBodies; Counter++)
        {
            RigidBody.Configuration Source =
                    Bodies.get(Counter).configurations[SourceConfigurationIndex];
            RigidBody.Configuration Target =
                    Bodies.get(Counter).configurations[TargetConfigurationIndex];


            Target.CMPosition = Vector2.add(Source.CMPosition,
                    Vector2.multiply(DeltaTime,Source.CMVelocity));

            Target.CMVelocity = Vector2.add(Source.CMVelocity,
                    Vector2.multiply((DeltaTime * Bodies.get(Counter).OneOverMass), Source.CMForce));


            Target.Orientation = Source.Orientation +
                    DeltaTime * Source.AngularVelocity;

            Target.AngularVelocity = Source.AngularVelocity +
                    (DeltaTime * Bodies.get(Counter).OneOverCMMomentOfInertia) *
                            Source.Torque;
        }
    }

    //-------------------------------------------------------------------------------

    private boolean pointInPolygon(RigidBody.Configuration.BoundingBox Box, Vector2 point) {

        int i, j=4-1;
        boolean oddNodes=false;

        for (i=0; i<4; i++) {
            if (Box.vertices[i].y<point.y && Box.vertices[j].y>=point.y
                    ||  Box.vertices[j].y<point.y && Box.vertices[i].y>=point.y) {
                if (Box.vertices[i].x+(point.y-Box.vertices[i].y)/(Box.vertices[j].y-Box.vertices[i].y)*(Box.vertices[j].x-Box.vertices[i].x)<point.x) {
                    oddNodes=!oddNodes; }}
            j=i;
        }
        return oddNodes;
    }

    //-------------------------------------------------------------------------------

    private Vector2 getCollisionNormal(RigidBody.Configuration.BoundingBox Box, Vector2 point, Collision collision) {

        double min = MathTools.REAL_MAX;
        Vector2 normal = new Vector2();

        Vector2[] edges = new Vector2[4];
        edges[0] = Vector2.subtract(Box.vertices[1], Box.vertices[0]);
        edges[1] = Vector2.subtract(Box.vertices[2], Box.vertices[1]);
        edges[2] = Vector2.subtract(Box.vertices[3], Box.vertices[2]);
        edges[3] = Vector2.subtract(Box.vertices[0], Box.vertices[3]);

        for(int i = 0; i < 4; i++) {
            Vector2 v1 = Box.vertices[i];
            Vector2 v2 = Box.vertices[(i+1)%3];

            double length = Math.abs((v2.y - v1.y)*point.x - (v2.x-v1.x)*point.y + v2.x*v1.y - v2.y*v1.x)/
                    Math.sqrt((v2.y-v1.y)*(v2.y-v1.y) + (v2.x-v1.x)*(v2.x-v1.x));

            if(length < min) {
                min = length;
                normal = edges[i].GetPerpendicular().GetNormal();
            }
        }

        if(min > 1.0)
            collision.collisionState = CollisionState.Penetrating;

        return normal;
    }

    //-------------------------------------------------------------------------------

    private Vector2 getRelativeVelocity(RigidBody Body1, RigidBody Body2, Vector2 Position, int ConfigurationIndex) {

        RigidBody.Configuration Configuration = Body1.configurations[ConfigurationIndex];
        RigidBody.Configuration Configuration2 = Body2.configurations[ConfigurationIndex];

        Vector2 CMToCornerPerp = Vector2.subtract(Position,
               Configuration.CMPosition).GetPerpendicular();

        Vector2 Velocity1 = Vector2.add(Configuration.CMVelocity,
                Vector2.multiply(Configuration.AngularVelocity, CMToCornerPerp));

        Vector2 CMToCornerPerp2 = Vector2.subtract(Position,
                Configuration2.CMPosition).GetPerpendicular();

        Vector2 Velocity2 = Vector2.add(Configuration2.CMVelocity,
                Vector2.multiply(Configuration2.AngularVelocity, CMToCornerPerp2));

        return Vector2.subtract(Velocity1,Velocity2);
    }

    //-------------------------------------------------------------------------------

    private CollisionState CheckForCollisions(int ConfigurationIndex) {

        // be optimistic!
        collisionState = CollisionState.Clear;

        Collision state = new Collision();
        state.collisionState = CollisionState.Clear;

        for(int Body = 0; Body < NumberOfBodies; ++Body) {
            for(int Body2 = Body + 1; Body2 < NumberOfBodies; ++Body2) {

                if(SeparateAxes(ConfigurationIndex, Bodies.get(Body), Bodies.get(Body2)) == true) {

                    RigidBody.Configuration.BoundingBox Box1 = Bodies.get(Body).configurations[ConfigurationIndex].Box;
                    RigidBody.Configuration.BoundingBox Box2 = Bodies.get(Body2).configurations[ConfigurationIndex].Box;

                    boolean pointFound = false;

                    for(int i = 0; i < 4; i++) {
                        if(pointInPolygon(Box1, Box2.vertices[i])) {

                            pointFound = true;

                            Vector2 relative = getRelativeVelocity(Bodies.get(Body2), Bodies.get(Body),
                                                    Box2.vertices[i], ConfigurationIndex);

                            state.CollisionNormal = getCollisionNormal(Box1, Box2.vertices[i], state);

                            state.CollidingBodyIndex = Body2;
                            state.CollidingBodyIndex2 = Body;
                            state.CollidingCornerIndex = i;

                            if(state.CollisionNormal.DotProduct(relative) < 0 && state.collisionState != CollisionState.Penetrating) {
                                state.collisionState = CollisionState.Colliding;
                                break;
                            }
                        }

                        if(pointInPolygon(Box2, Box1.vertices[i])) {

                            pointFound = true;

                            Vector2 relative = getRelativeVelocity(Bodies.get(Body), Bodies.get(Body2),
                                    Box1.vertices[i], ConfigurationIndex);

                            state.CollisionNormal = getCollisionNormal(Box2, Box1.vertices[i], state);

                            state.CollidingBodyIndex = Body;
                            state.CollidingBodyIndex2 = Body2;
                            state.CollidingCornerIndex = i;

                            if(state.CollisionNormal.DotProduct(relative) < 0 && state.collisionState != CollisionState.Penetrating) {
                                state.collisionState = CollisionState.Colliding;
                                break;
                            }
                        }
                    }

                    if(!pointFound || (pointFound && state.collisionState == CollisionState.Clear)) {
                        state.CollisionNormal = getCollisionNormal(Box2, Box1.vertices[0], state);

                        state.CollidingBodyIndex = Body;
                        state.CollidingBodyIndex2 = Body2;
                        state.CollidingCornerIndex = 0;

                        state.collisionState = CollisionState.Penetrating;
                    }
                    state.collisionType = CollisionType.BoxBox;

                    if(state.collisionState == CollisionState.Penetrating) {
                        collisionState = CollisionState.Penetrating;
                    }
                    else if(collisionState == CollisionState.Clear)
                        collisionState = state.collisionState;

                    collisions.add(state);

                    state = new Collision();
                    state.collisionState = CollisionState.Clear;
                }

            }
        }

        final float DepthEpsilon = 0.01f;

        final double HalfWidth = WorldWidth / 2.0f;
        final double HalfHeight = WorldHeight / 2.0f;

        for(int Body = 0; Body < NumberOfBodies; Body++)
        {
            // @todo active configuration number?!?!?
            RigidBody.Configuration Configuration =
                    Bodies.get(Body).configurations[ConfigurationIndex];

            RigidBody.Configuration.BoundingBox Box =
                Configuration.Box;

            for(int Counter = 0; Counter < 4; Counter++)
            {
                Vector2 Position = Box.vertices[Counter];

                Vector2 CMToCornerPerp =
                        Vector2.subtract(Position, Configuration.CMPosition).GetPerpendicular();

                Vector2 Velocity = Vector2.add(Configuration.CMVelocity,
                        Vector2.multiply(Configuration.AngularVelocity, CMToCornerPerp));

                for(int WallIndex = 0; WallIndex < NumberOfWalls; WallIndex++)
                {
                    Wall wall = Walls[WallIndex];

                    double axbyc = Position.DotProduct(wall.Normal) + wall.c;

                    if(axbyc < -DepthEpsilon)
                    {
                        collisionState = CollisionState.Penetrating;
                        state.collisionState = CollisionState.Penetrating;
                        state.CollisionNormal = wall.Normal;
                        state.CollidingCornerIndex = Counter;
                        state.CollidingBodyIndex = Body;
                        state.collisionType = CollisionType.BoxWall;
                        collisions.add(state);

                        state = new Collision();
                        state.collisionState = CollisionState.Clear;
                    }
                    else
                    if(axbyc < DepthEpsilon)
                    {
                        double RelativeVelocity = wall.Normal.DotProduct(Velocity);

                        if(RelativeVelocity < 0)
                        {
                            if(collisionState != CollisionState.Penetrating)
                                collisionState = CollisionState.Colliding;

                            state.collisionState = CollisionState.Colliding;
                            state.CollisionNormal = wall.Normal;
                            state.CollidingCornerIndex = Counter;
                            state.CollidingBodyIndex = Body;
                            state.collisionType = CollisionType.BoxWall;
                            collisions.add(state);

                            state = new Collision();
                            state.collisionState = CollisionState.Clear;
                        }
                    }
                }
            }
        }

        return collisionState;
    }

    //-------------------------------------------------------------------------------

    private boolean SeparateAxes(int ConfigurationIndex, RigidBody Body1, RigidBody Body2) {

        RigidBody.Configuration Configuration =
                Body1.configurations[ConfigurationIndex];

        RigidBody.Configuration.BoundingBox Box =
                Configuration.Box;

        RigidBody.Configuration Configuration2 =
                Body2.configurations[ConfigurationIndex];

        RigidBody.Configuration.BoundingBox Box2 =
                Configuration2.Box;

        Vector2[] axes = new Vector2[4];

        axes[0] = Vector2.subtract(Box.vertices[0], Box.vertices[3]);
        axes[1] = Vector2.subtract(Box.vertices[0], Box.vertices[1]);
        axes[2] = Vector2.subtract(Box2.vertices[0], Box2.vertices[3]);
        axes[3] = Vector2.subtract(Box2.vertices[0], Box2.vertices[1]);

        for(int axe = 0; axe < 4; axe++) {

            Vector2[] ProjectedPointsBox1 = new Vector2[4];
            Vector2[] ProjectedPointsBox2 = new Vector2[4];

            for(int vertex = 0; vertex < 4; vertex++) {
                double xp = Box.vertices[vertex].x;
                double yp = Box.vertices[vertex].y;

                double xa = axes[axe].x;
                double ya = axes[axe].y;

                double x,y;

                if(ya != 0 && xa != 0) {
                    x = (yp + xa / ya * xp) * xa * ya / (xa * xa + ya * ya);
                    y = ya / xa * x;
                }
                else if(xa == 0 && ya == 0) {
                    return false;
                }
                else if(ya == 0) {
                    y = 0;
                    x = xp;
                }
                else {
                    x = 0;
                    y = yp;
                }

                ProjectedPointsBox1[vertex] = new Vector2(x,y);

                xp = Box2.vertices[vertex].x;
                yp = Box2.vertices[vertex].y;

                if(ya != 0 && xa != 0) {
                    x = (yp + xa / ya * xp) * xa * ya / (xa * xa + ya * ya);
                    y = ya / xa * x;
                }
                else if(xa == 0 && ya == 0) {
                    return false;
                }
                else if(ya == 0) {
                    y = 0;
                    x = xp;
                }
                else {
                    x = 0;
                    y = yp;
                }

                ProjectedPointsBox2[vertex] = new Vector2(x,y);
            }

            double Box1Max = ProjectedPointsBox1[0].DotProduct(axes[axe]);
            double Box1Min = Box1Max;
            double Box2Max = ProjectedPointsBox2[0].DotProduct(axes[axe]);
            double Box2Min = Box2Max;

            for(int i = 1; i < 4; i++) {
                double tmp = ProjectedPointsBox1[i].DotProduct(axes[axe]);
                if(tmp > Box1Max)
                    Box1Max = tmp;
                if(tmp < Box1Min)
                    Box1Min = tmp;

                tmp = ProjectedPointsBox2[i].DotProduct(axes[axe]);
                if(tmp > Box2Max)
                    Box2Max = tmp;
                if(tmp < Box2Min)
                    Box2Min = tmp;
            }

            if(!((Box2Min <= Box1Min && Box2Max >= Box1Min)
                    || (Box2Min <= Box1Max && Box2Max >= Box1Max)
                    || (Box2Min >= Box1Min && Box2Max <= Box1Max)
                    || (Box1Min >= Box2Min && Box1Max <= Box2Max)))
                return false;
        }

        return true;
    }

    //-------------------------------------------------------------------------------

    private void ResolveCollisions(int ConfigurationIndex) {

        for(int i = 0; i < collisions.size(); i++) {
            Collision state = collisions.get(i);
            if(state.collisionState != CollisionState.Colliding)
                continue;

            if(state.collisionType == CollisionType.BoxBox) {

                RigidBody Body = Bodies.get(state.CollidingBodyIndex);
                RigidBody.Configuration Configuration =
                        Body.configurations[ConfigurationIndex];

                RigidBody Body2 = Bodies.get(state.CollidingBodyIndex2);
                RigidBody.Configuration Configuration2 =
                        Body2.configurations[ConfigurationIndex];

                Vector2 Position =
                        Configuration.Box.vertices[state.CollidingCornerIndex];

                Vector2 CMToCornerPerp = Vector2.subtract(Position,
                        Configuration.CMPosition).GetPerpendicular();

                Vector2 Velocity1 = Vector2.add(Configuration.CMVelocity,
                        Vector2.multiply(Configuration.AngularVelocity, CMToCornerPerp));

                Vector2 CMToCornerPerp2 = Vector2.subtract(Position,
                        Configuration2.CMPosition).GetPerpendicular();

                Vector2 Velocity2 = Vector2.add(Configuration2.CMVelocity,
                        Vector2.multiply(Configuration2.AngularVelocity, CMToCornerPerp2));

                Vector2 Velocity = Vector2.subtract(Velocity1,Velocity2);

                double ImpulseNumerator = -(1 + Body.CoefficientOfRestitution) *
                        Velocity.DotProduct(state.CollisionNormal);

                double PerpDot = CMToCornerPerp.DotProduct(state.CollisionNormal);

                double PerpDot2 = CMToCornerPerp2.DotProduct(state.CollisionNormal);

                double ImpulseDenominator =
                        (Body.OneOverMass + Body2.OneOverMass) +
                                Body.OneOverCMMomentOfInertia * PerpDot * PerpDot +
                                Body2.OneOverCMMomentOfInertia * PerpDot2 * PerpDot2;

                double Impulse = ImpulseNumerator / ImpulseDenominator;

                if(state.CollisionNormal.DotProduct(Configuration.CMForce) < 0 && Math.abs(Impulse) < 10)
                {
                    //Configuration.CMVelocity.multiply(Math.abs(Impulse)/15.);
                    //Configuration.AngularVelocity *= Math.abs(Impulse)/15.;
                }

                if(state.CollisionNormal.DotProduct(Configuration2.CMForce) < 0 && Math.abs(Impulse) < 10)
                {
                    //Configuration2.CMVelocity.multiply(Math.abs(Impulse)/15.);
                    //Configuration2.AngularVelocity *= Math.abs(Impulse)/15.;
                }

                Configuration.CMVelocity.add(Vector2.multiply(Impulse * Body.OneOverMass, state.CollisionNormal));

                Configuration.AngularVelocity +=
                        Impulse * Body.OneOverCMMomentOfInertia * PerpDot;

                Configuration2.CMVelocity.subtract(Vector2.multiply(Impulse * Body2.OneOverMass, state.CollisionNormal));

                Configuration2.AngularVelocity -=
                        Impulse * Body2.OneOverCMMomentOfInertia * PerpDot2;

                return;
            }

            RigidBody Body = Bodies.get(state.CollidingBodyIndex);
            RigidBody.Configuration Configuration =
                    Body.configurations[ConfigurationIndex];

            Vector2 Position =
                    Configuration.Box.vertices[state.CollidingCornerIndex];

            Vector2 CMToCornerPerp = Vector2.subtract(Position,
                    Configuration.CMPosition).GetPerpendicular();

            Vector2 Velocity = Vector2.add(Configuration.CMVelocity,
                    Vector2.multiply(Configuration.AngularVelocity, CMToCornerPerp));

            double ImpulseNumerator = -(1 + Body.CoefficientOfRestitution) *
                    Velocity.DotProduct(state.CollisionNormal);

            double PerpDot = CMToCornerPerp.DotProduct(state.CollisionNormal);

            double ImpulseDenominator = Body.OneOverMass +
                    Body.OneOverCMMomentOfInertia * PerpDot * PerpDot;

            double Impulse = ImpulseNumerator / ImpulseDenominator;

            if(state.CollisionNormal.DotProduct(Configuration.CMForce) < 0 && Math.abs(Impulse) < 10)
            {
                //Configuration.CMVelocity.multiply(Math.abs(Impulse)/15.);
                //Configuration.AngularVelocity *= Math.abs(Impulse)/15.;

                //Configuration.CMVelocity.subtract(Vector2.multiply(state.CollisionNormal.DotProduct(Velocity), state.CollisionNormal));
            }

            Configuration.CMVelocity.add(Vector2.multiply(Impulse * Body.OneOverMass, state.CollisionNormal));
            Configuration.AngularVelocity +=
                    Impulse * Body.OneOverCMMomentOfInertia * PerpDot;
        }
    }

    //-------------------------------------------------------------------------------

    private void CalculateVertices(int ConfigurationIndex) {

        for(int Counter = 0; Counter < NumberOfBodies; Counter++)
        {
            final Matrix2 Rotation = new Matrix2(
                Bodies.get(Counter).configurations[ConfigurationIndex].
                        Orientation);

            final Vector2 Position =
                Bodies.get(Counter).configurations[ConfigurationIndex].
                        CMPosition;

            RigidBody.Configuration.BoundingBox Box =
                Bodies.get(Counter).configurations[ConfigurationIndex].Box;

            double Width = Bodies.get(Counter).Width / 2.0f;
            double Height = Bodies.get(Counter).Height / 2.0f;

            Box.vertices[0] = Vector2.add(Position, Rotation.multiply(new Vector2(Width, Height)));
            Box.vertices[1] = Vector2.add(Position, Rotation.multiply(new Vector2(Width, -Height)));
            Box.vertices[2] = Vector2.add(Position, Rotation.multiply(new Vector2(-Width, -Height)));
            Box.vertices[3] = Vector2.add(Position, Rotation.multiply(new Vector2(-Width, Height)));
        }
    }
}
