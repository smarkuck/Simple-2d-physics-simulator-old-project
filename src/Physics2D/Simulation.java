package Physics2D;

import java.util.ArrayList;

public class Simulation {

    public RigidBody addRigidBody(double Density, double Width, double Height, double CoefficientOfRestitution, boolean isStatic) {
        RigidBody body = new RigidBody(Density, Width, Height, CoefficientOfRestitution, isStatic);
        Bodies.add(body);
        ++NumberOfBodies;

        return body;
    }

    //-------------------------------------------------------------------------------

    public Simulation removeRigidBody(RigidBody body) {
        Bodies.remove(body);
        --NumberOfBodies;
        return this;
    }

    //-------------------------------------------------------------------------------

    public RigidBody getRigidBody(int index) {
        return Bodies.get(index);
    }

    //-------------------------------------------------------------------------------

    public Force addForce(Vector2 position, double value, double range, boolean isConstant) {

        Force f = new Force();

        f.Position = position;
        f.isConstant = isConstant;
        f.value = value;

        if(range <= 0)
            f.range = 0.1;
        else
            f.range = range;

        Forces.add(f);

        return f;
    }

    //-------------------------------------------------------------------------------

    public Simulation removeForce(Force force) {
        Forces.remove(force);
        return this;
    }

    //-------------------------------------------------------------------------------

    public Force getForce(int index) {
        return Forces.get(index);
    }

    //-------------------------------------------------------------------------------

    public Spring addSpring(Vector2 position, RigidBody body, int vertex, double hooke, double damping) {
        Spring spr = new Spring(position, body, vertex, hooke, damping);
        Springs.add(spr);
        return spr;
    }

    //-------------------------------------------------------------------------------

    public Spring addSpring(RigidBody body1, int vertex1, RigidBody body2, int vertex2, double hooke, double damping) {
        Spring spr = new Spring(body1, vertex1, body2, vertex2, hooke, damping);
        Springs.add(spr);
        return spr;
    }

    //-------------------------------------------------------------------------------

    public Simulation removeSpring(Spring spring) {
        Springs.remove(spring);
        return this;
    }

    //-------------------------------------------------------------------------------

    public Spring getSpring(int index) {
        return Springs.get(index);
    }

    //-------------------------------------------------------------------------------

    public boolean isGravityActive() {
        return GravityActive;
    }

    //-------------------------------------------------------------------------------

    public Simulation enableGravity(boolean enable) {
        GravityActive = enable;
        return this;
    }

    //-------------------------------------------------------------------------------

    public Simulation setGravity(Vector2 gravity) {
        this.Gravity.x = gravity.x;
        this.Gravity.y = gravity.y;
        return this;
    }

    //-------------------------------------------------------------------------------

    public boolean isBorder() {
        return isBorder;
    }

    //-------------------------------------------------------------------------------

    public Simulation enableBorder(boolean enable) {
        isBorder = enable;
        return this;
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

        // inicjalizuj ściany
        Walls[0].Normal = new Vector2(0, -1);
        Walls[0].c = WorldHeight/2;

        Walls[1].Normal = new Vector2(0, 1);
        Walls[1].c = WorldHeight/2;

        Walls[2].Normal = new Vector2(-1 ,0);
        Walls[2].c = WorldWidth/2;

        Walls[3].Normal = new Vector2(1 ,0);
        Walls[3].c = WorldWidth/2;

        // generuj linie ścian
        for(int Counter = 0; Counter < NumberOfWalls; Counter++)
        {
            Wall wall = Walls[Counter];

            // stwórz długą linię w kierunku sciany

            Vector2 PointOnWall = Vector2.multiply(-wall.c, wall.Normal);
            Vector2 v = wall.Normal.GetPerpendicular();
            double t0 = -MathTools.REAL_MAX;
            double t1 = MathTools.REAL_MAX;

            // utnij linie na przecięciach

            for(int WallIndex = 0; WallIndex < NumberOfWalls; WallIndex++)
            {
                if(WallIndex != Counter)
                {
                    Wall clipWall = Walls[WallIndex];

                    double Denominator = v.DotProduct(clipWall.Normal);

                    if(Math.abs(Denominator) > MathTools.Epsilon)
                    {
                        double t = - (clipWall.c +
                                PointOnWall.DotProduct(clipWall.Normal)) /
                                Denominator;

                        if(Denominator > 0)
                        {
                            // linia przecina stronę t0
                            if(t > t0)
                            {
                                t0 = t;
                            }
                        }
                        else
                        {
                            // linia przecina stronę t1
                            if(t < t1)
                            {
                                t1 = t;
                            }
                        }
                    }
                }
            }

            wall.StartPoint = Vector2.add(PointOnWall, Vector2.multiply(t0,v));
            wall.EndPoint = Vector2.add(PointOnWall, Vector2.multiply(t1,v));
        }

        // oblicz początkową pozycję wierzchołków
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
                // za duży skok czasowy, zmniejszanie
                TargetTime = (CurrentTime + TargetTime) / 2.0;

                if(Math.abs(TargetTime - CurrentTime) < 0.0001) {
                    ++error;
                    double move = error / 100.;

                    for(int i = 0; i < collisions.size(); i++) {
                        Collision state = collisions.get(i);
                        if(state.collisionState != CollisionState.Penetrating)
                            continue;

                        if(state.collisionType == CollisionType.BoxBox) {

                            RigidBody Body1 = Bodies.get(state.CollidingBodyIndex);
                            RigidBody Body2 = Bodies.get(state.CollidingBodyIndex2);

                            if(Body2.isStatic) {
                                RigidBody tmp = Body1;
                                Body1 = Body2;
                                Body2 = tmp;
                            }

                            RigidBody.Configuration conf = Body1.configurations[SourceConfigurationIndex];
                            RigidBody.Configuration conf2 = Body2.configurations[SourceConfigurationIndex];

                            if(Body1.isStatic) {
                                if (conf.CMPosition.x > conf2.CMPosition.x)
                                    conf2.CMPosition.x -= move;
                                else {
                                    conf2.CMPosition.x += move;
                                }

                                if (conf.CMPosition.y > conf2.CMPosition.y) {
                                    conf2.CMPosition.y -= move;
                                } else {
                                    conf2.CMPosition.y += move;
                                }
                            } else {
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
                        }
                        else {
                            RigidBody.Configuration conf = Bodies.get(state.CollidingBodyIndex).configurations[SourceConfigurationIndex];

                            conf.CMPosition.add(Vector2.multiply(move,state.CollisionNormal));
                        }
                    }
                }
                collisions.clear();
            }
            else
            {
                error = 0;
                // albo wystąpiła kolizja albo nie

                if(collisionState == CollisionState.Colliding)
                {
                    int Counter = 0;
                    do
                    {
                        ResolveCollisions(TargetConfigurationIndex);
                        collisions.clear();
                        Counter++;
                    } while((CheckForCollisions(TargetConfigurationIndex) ==
                            CollisionState.Colliding) && (Counter < 100));

                }

                // zamiana konfiguracji docelowej na źródło

                CurrentTime = TargetTime;
                TargetTime = DeltaTime;

                for(int i = 0; i < Bodies.size(); i++) {
                    RigidBody Body = Bodies.get(i);
                    RigidBody.Configuration tmp = Body.configurations[SourceConfigurationIndex];
                    Body.configurations[SourceConfigurationIndex] = Body.configurations[TargetConfigurationIndex];
                    Body.configurations[TargetConfigurationIndex] = tmp;
                }

                //SourceConfigurationIndex = (SourceConfigurationIndex == 1) ? 0 : 1;
                //TargetConfigurationIndex = (TargetConfigurationIndex == 1) ? 0 : 1;
            }
        }
    }

    //-------------------------------------------------------------------------------

    double WorldWidth, WorldHeight;

    static final int NumberOfWalls = 4;
    Wall[] Walls = new Wall[NumberOfWalls];

    ArrayList<Force> Forces = new ArrayList<Force>();
    ArrayList<Spring> Springs = new ArrayList<Spring>();

    int NumberOfBodies = 0;
    ArrayList<RigidBody> Bodies = new ArrayList<RigidBody>();

    boolean GravityActive = true;
    Vector2 Gravity = new Vector2(0, -100);

    boolean isBorder = true;

    int error = 0;

    CollisionState collisionState;

    ArrayList<Collision> collisions = new ArrayList<Collision>();

    int SourceConfigurationIndex;
    int TargetConfigurationIndex;

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

    //-------------------------------------------------------------------------------

    private void ComputeForces(int ConfigurationIndex) {
        int Counter;

        for(Counter = 0; Counter < NumberOfBodies; Counter++)
        {
            RigidBody Body = Bodies.get(Counter);

            if(Body.isStatic)
                continue;

            RigidBody.Configuration Configuration =
                    Body.configurations[ConfigurationIndex];

            // wyczyść siły

            Configuration.Torque = 0;
            Configuration.CMForce = new Vector2(0 ,0);

            if(GravityActive)
            {
                Configuration.CMForce.add(Vector2.divide(Gravity,Body.OneOverMass));
            }

            for(int i = 0; i < Forces.size(); i++) {

                Force data = Forces.get(i);

                Vector2 distance = Vector2.subtract(Configuration.CMPosition, data.Position);
                if(distance.GetLength() > data.range || distance.GetLength() == 0)
                    continue;

                Vector2 computedForce = distance.GetNormal().multiply(data.value);
                if(!data.isConstant)
                    computedForce.multiply((data.range - distance.GetLength())/data.range*0.9+0.1);

                if(distance.GetNormal().DotProduct(Configuration.CMVelocity) > 0 && data.value < 0)
                    computedForce.multiply(1.046);

                if(distance.GetNormal().DotProduct(Configuration.CMVelocity) < 0 && data.value > 0)
                    computedForce.multiply(1.0588);

                Configuration.CMForce.add(computedForce);
            }
        }

        for(int i = 0; i < Springs.size(); i++) {
            Spring spr = Springs.get(i);

            if(spr.toWall == true) {

                RigidBody Body = spr.body2;
                RigidBody.Configuration Configuration =
                        Body.configurations[ConfigurationIndex];

                Vector2 Position = Configuration.Box.vertices[spr.vertex2];

                Vector2 U = Vector2.subtract(Position, Configuration.CMPosition);
                Vector2 VU = Vector2.add(Configuration.CMVelocity,
                        Vector2.multiply(Configuration.AngularVelocity, U.GetPerpendicular()));

                Vector2 Spring = Vector2.multiply(-spr.Hooke, Vector2.subtract(Position, spr.pos));
                // wytłumianie sprężyny

                Vector2 DampingForce =
                        Vector2.multiply(-spr.damping * (VU.DotProduct(Spring)/Spring.DotProduct(Spring)), Spring);

                Spring.add(DampingForce);

                if(!Double.isNaN(Spring.x) && !Double.isNaN(Spring.y)) {
                    Configuration.CMForce.add(Spring);
                    Configuration.Torque += U.PerpDotProduct(Spring);
                }
            }
            else {
                RigidBody Body0 = spr.body1;
                RigidBody.Configuration Configuration0 =
                        Body0.configurations[ConfigurationIndex];

                Vector2 Position0 = Configuration0.Box.vertices[spr.vertex1];
                Vector2 U0 = Vector2.subtract(Position0, Configuration0.CMPosition);
                Vector2 VU0 = Vector2.add(Configuration0.CMVelocity,
                        Vector2.multiply(Configuration0.AngularVelocity,U0.GetPerpendicular()));

                RigidBody Body1 = spr.body2;
                RigidBody.Configuration Configuration1 =
                        Body1.configurations[ConfigurationIndex];

                Vector2 Position1 = Configuration1.Box.vertices[spr.vertex2];
                Vector2 U1 = Vector2.subtract(Position1,Configuration1.CMPosition);
                Vector2 VU1 = Vector2.add(Configuration1.CMVelocity,
                        Vector2.multiply(Configuration1.AngularVelocity, U1.GetPerpendicular()));

                Vector2 SpringVector = Vector2.subtract(Position1, Position0);
                Vector2 Spring = Vector2.multiply(-spr.Hooke,SpringVector);

                Vector2 RelativeVelocity = Vector2.subtract(VU1, VU0);
                // wytłumianie sprężyny
                Vector2 DampingForce =
                        Vector2.multiply(-spr.damping * (RelativeVelocity.DotProduct(SpringVector)/
                                SpringVector.DotProduct(SpringVector)), SpringVector);

                Spring.add(DampingForce);

                if(!Double.isNaN(Spring.x) && !Double.isNaN(Spring.y)) {
                    Configuration0.CMForce.subtract(Spring);
                    Configuration0.Torque -= U0.PerpDotProduct(Spring);

                    Configuration1.CMForce.add(Spring);
                    Configuration1.Torque += U1.PerpDotProduct(Spring);
                }
            }
        }
    }

    //-------------------------------------------------------------------------------

    private void Integrate(double DeltaTime) {
        int Counter;

        for(Counter = 0; Counter < NumberOfBodies; Counter++)
        {
            if(Bodies.get(Counter).isStatic)
                continue;

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

        collisionState = CollisionState.Clear;

        Collision state = new Collision();
        state.collisionState = CollisionState.Clear;

        for(int Body = 0; Body < NumberOfBodies; ++Body) {
            for(int Body2 = Body + 1; Body2 < NumberOfBodies; ++Body2) {

                if(Bodies.get(Body).isStatic && Bodies.get(Body2).isStatic)
                    continue;

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

        if(isBorder == false)
            return collisionState;

        final float DepthEpsilon = 0.01f;

        for(int Body = 0; Body < NumberOfBodies; Body++)
        {
            if(Bodies.get(Body).isStatic)
                continue;

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

            if(state.collisionType == CollisionType.BoxWall) {

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

                Configuration.CMVelocity.add(Vector2.multiply(Impulse * Body.OneOverMass, state.CollisionNormal));
                Configuration.AngularVelocity +=
                        Impulse * Body.OneOverCMMomentOfInertia * PerpDot;
            }

            else if(state.collisionType == CollisionType.BoxBox) {

                RigidBody Body = Bodies.get(state.CollidingBodyIndex);
                RigidBody Body2 = Bodies.get(state.CollidingBodyIndex2);

                if(Body.isStatic) {
                    RigidBody tmp = Body;
                    Body = Body2;
                    Body2 = tmp;
                }

                RigidBody.Configuration Configuration =
                        Body.configurations[ConfigurationIndex];

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
                double PerpDot2 = 0;

                double ImpulseDenominator;

                if(!Body2.isStatic) {
                    PerpDot2 = CMToCornerPerp2.DotProduct(state.CollisionNormal);

                    ImpulseDenominator =
                            (Body.OneOverMass + Body2.OneOverMass) +
                                    Body.OneOverCMMomentOfInertia * PerpDot * PerpDot +
                                    Body2.OneOverCMMomentOfInertia * PerpDot2 * PerpDot2;
                }
                else
                    ImpulseDenominator = Body.OneOverMass +
                            Body.OneOverCMMomentOfInertia * PerpDot * PerpDot;

                double Impulse = ImpulseNumerator / ImpulseDenominator;

                Configuration.CMVelocity.add(Vector2.multiply(Impulse * Body.OneOverMass, state.CollisionNormal));

                Configuration.AngularVelocity +=
                        Impulse * Body.OneOverCMMomentOfInertia * PerpDot;

                if(!Body2.isStatic) {
                    Configuration2.CMVelocity.subtract(Vector2.multiply(Impulse * Body2.OneOverMass, state.CollisionNormal));

                    Configuration2.AngularVelocity -=
                            Impulse * Body2.OneOverCMMomentOfInertia * PerpDot2;
                }
            }

            return;
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
