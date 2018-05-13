import java.awt.*;
import java.awt.event.*;
import java.util.concurrent.TimeUnit;

class Example extends Frame {

    Simulation world;
    int width;
    int height;

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

        Example demo = new Example(400, 400);

        demo.world.addRigidBody(0.01, 40, 20, 1);
        demo.world.addRigidBody(0.01, 50, 10, 1);
        demo.world.addRigidBody(0.01, 10, 100, 0.5);

        //demo.world.Bodies.get(0).configurations[0].CMPosition = new Vector2(0, 100);

        demo.world.Bodies.get(0).configurations[0].CMPosition = new Vector2(-100, 0);
        demo.world.Bodies.get(1).configurations[0].CMPosition = new Vector2(100, 0);

        demo.world.Bodies.get(0).configurations[0].CMVelocity = new Vector2(100, 0);
        demo.world.Bodies.get(1).configurations[0].CMVelocity = new Vector2(-100, 0);

        //demo.world.Bodies.get(0).configurations[0].CMPosition = new Vector2(100, 100);
        //demo.world.Bodies.get(0).configurations[0].CMVelocity = new Vector2(40 ,10);
        //demo.world.Bodies.get(0).configurations[0].AngularVelocity = MathTools.PI;

        //demo.world.Bodies.get(2).configurations[0].CMPosition = new Vector2(-100, 100);
        //demo.world.Bodies.get(2).configurations[0].Orientation = 2;

        double fps = 1000/60.;

        //draw 60 frames per second
        while(true) {

            long startRendering=System.nanoTime();

            demo.repaint();
            demo.paint(demo.getGraphics());

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
        Graphics2D g2 = (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        renderWorld();
    }

    private void renderWorld() {
        Graphics2D g = (Graphics2D)getGraphics();

        for(int i = 0; i < world.NumberOfWalls; i++) {
            int x1, x2, y1, y2;

            x1 = (int)world.Walls[i].StartPoint.x;
            y1 = (int)world.Walls[i].StartPoint.y;
            x2 = (int)world.Walls[i].EndPoint.x;
            y2 = (int)world.Walls[i].EndPoint.y;

            g.drawLine(x1+250, -y1+250, x2+250, -y2+250);
        }

        for(int i = 0; i < world.NumberOfBodies; i++) {
            int x1, x2, y1, y2;

            x1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[0].x;
            y1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[0].y;

            x2 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[1].x;
            y2 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[1].y;

            g.drawLine(x1+250, -y1+250, x2+250, -y2+250);

            x1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[2].x;
            y1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[2].y;

            g.drawLine(x2+250, -y2+250, x1+250, -y1+250);

            x2 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[3].x;
            y2 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[3].y;

            g.drawLine(x1+250, -y1+250, x2+250, -y2+250);

            x1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[0].x;
            y1 = (int)world.Bodies.get(i).configurations[world.SourceConfigurationIndex].Box.vertices[0].y;

            g.drawLine(x2+250, -y2+250, x1+250, -y1+250);
        }
    }
}
