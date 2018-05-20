import Physics2D.*;
import com.sun.corba.se.impl.orbutil.graph.Graph;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

class Example extends Frame {

    Simulation world;
    int width;
    int height;

    ArrayList<RigidBody> Bodies = new ArrayList<RigidBody>();
    ArrayList<Force> Forces = new ArrayList<Force>();
    ArrayList<Spring> Springs = new ArrayList<Spring>();

    //-------------------------------------------------------------------------------

    public Example(int width, int height){
        super("Physics 2D Example");

        this.width = width;
        this.height = height;
        world = new Simulation(width, height);

        prepareGUI();
        setVisible(true);
    }

    //-------------------------------------------------------------------------------

    public static void main(String[] args){

        Example demo = new Example(600, 800);

        for(int i = 0; i < 5; i++) {
            demo.Bodies.add(demo.world.addRigidBody(0.01, 10, 10, 1.0, false).
                    setCMPosition(new Vector2(40 - i*20,-i*20+250)));
        }

        for(int i = 0; i < 5; i++) {
            demo.Bodies.add(demo.world.addRigidBody(0.05, 20, 20, 1.0, false).
                    setCMPosition(new Vector2(-40 + i*20,50)));
        }

        demo.Forces.add(demo.world.addForce(new Vector2(0, 200), -1000, 300, false));

        demo.Bodies.add(demo.world.addRigidBody(0.01, 600, 10, 1, true));

        for(int i = 0; i < 4; i++)
            demo.Bodies.add(demo.world.addRigidBody(0.01, 50, 10, 1.0, false).
                setCMPosition(new Vector2(i*100-125, -300)));

        demo.Springs.add(demo.world.addSpring(new Vector2(-150, -300), demo.Bodies.get(11), 2, 30, 15));
        demo.Springs.add(demo.world.addSpring(demo.Bodies.get(11), 3, demo.Bodies.get(12), 2, 30, 15));
        demo.Springs.add(demo.world.addSpring(demo.Bodies.get(12), 3, demo.Bodies.get(13), 2, 30, 15));
        demo.Springs.add(demo.world.addSpring(demo.Bodies.get(13), 3, demo.Bodies.get(14), 2, 30, 15));
        demo.Springs.add(demo.world.addSpring(new Vector2(150, -300), demo.Bodies.get(14), 3, 30, 15));

        demo.Forces.add(demo.world.addForce(new Vector2(0, -350), 1000, 300, false));
        demo.Forces.add(demo.world.addForce(new Vector2(100, -300), 1000, 300, false));

        demo.world.enableGravity(false).enableBorder(false);

        double fps = 1000/1000.;
        Graphics g = demo.getGraphics();

        //draw 60 frames per second
        while(true) {

            long startRendering=System.nanoTime();

            //demo.repaint();
            demo.paint(g);

            long durationMs;
            // now waits
            do
            {
                //duration of the frame rendering in ms :
                durationMs=TimeUnit.NANOSECONDS.toMillis(System.nanoTime()-startRendering);
            } while (durationMs < fps);

            demo.world.Simulate(fps/1000.);
        }
    }

    //-------------------------------------------------------------------------------

    private void prepareGUI(){
        setSize(this.width + 100,this.height + 100);
        addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent windowEvent){
                System.exit(0);
            }
        });
    }

    //-------------------------------------------------------------------------------

    @Override
    public void paint(Graphics g) {
        super.paint(g);

        Graphics2D g2 = (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        renderWorld();
    }

    private void renderWorld() {
        Graphics2D g = (Graphics2D)getGraphics();

        int halfWidth = (width + 100)/2;
        int halfHeight = (height + 100)/2;

        g.clearRect(0, 0, width + 100, height + 100);

        if(world.isBorder()) {
            g.drawLine(50, height+50, 50+width, height+50);
            g.drawLine(50, 50, 50+width, 50);
            g.drawLine(50, 50, 50, height+50);
            g.drawLine(50+width, 50, 50+width, height+50);
        }

        for(int i = 0; i < Forces.size(); i++) {
            int x = (int)Forces.get(i).getPosition().x;
            int y = (int)Forces.get(i).getPosition().y;

            g.drawLine(x+halfWidth-5, -y+halfHeight, x+halfWidth+5, -y+halfHeight);
            g.drawLine(x+halfWidth, -y+halfHeight-5, x+halfWidth, -y+halfHeight+5);
        }

        for(int i = 0; i < Springs.size(); i++) {

            Spring spr = Springs.get(i);
            int x1, x2, y1, y2;

            x1 = (int)spr.getAttachment1().x;
            y1 = (int)spr.getAttachment1().y;

            x2 = (int)spr.getAttachment2().x;
            y2 = (int)spr.getAttachment2().y;

            g.drawLine(x1+halfWidth, -y1+halfHeight, x2+halfWidth, -y2+halfHeight);
        }

        for(int i = 0; i < Bodies.size(); i++) {
            int x1, x2, y1, y2;

            RigidBody body = Bodies.get(i);

            g.setPaint(Color.blue);

            int x[] = {(int)body.getVertex(1).x+halfWidth,
                        (int)body.getVertex(2).x+halfWidth,
                        (int)body.getVertex(3).x+halfWidth,
                        (int)body.getVertex(4).x+halfWidth};

            int y[] = {-(int)body.getVertex(1).y+halfHeight,
                    -(int)body.getVertex(2).y+halfHeight,
                    -(int)body.getVertex(3).y+halfHeight,
                    -(int)body.getVertex(4).y+halfHeight};

            g.fillPolygon(x, y, 4);

//            x1 = (int)body.getVertex(1).x;
//            y1 = (int)body.getVertex(1).y;
//
//            x2 = (int)body.getVertex(2).x;
//            y2 = (int)body.getVertex(2).y;
//
//            g.drawLine(x1+halfWidth, -y1+halfHeight, x2+halfWidth, -y2+halfHeight);
//
//            x1 = (int)body.getVertex(3).x;
//            y1 = (int)body.getVertex(3).y;
//
//            g.drawLine(x2+halfWidth, -y2+halfHeight, x1+halfWidth, -y1+halfHeight);
//
//            x2 = (int)body.getVertex(4).x;
//            y2 = (int)body.getVertex(4).y;
//
//            g.drawLine(x1+halfWidth, -y1+halfHeight, x2+halfWidth, -y2+halfHeight);
//
//            x1 = (int)body.getVertex(1).x;
//            y1 = (int)body.getVertex(1).y;
//
//            g.drawLine(x2+halfWidth, -y2+halfHeight, x1+halfWidth, -y1+halfHeight);
        }
    }
}
