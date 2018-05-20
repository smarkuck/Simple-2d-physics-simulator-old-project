import Physics2D.*;
import org.lwjgl.*;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.*;
import org.lwjgl.system.*;

import java.nio.*;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import static org.lwjgl.glfw.Callbacks.*;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.system.MemoryStack.*;
import static org.lwjgl.system.MemoryUtil.*;

public class Example {

    // The window handle
    private long window;

    Simulation world;
    int width;
    int height;

    ArrayList<RigidBody> Bodies = new ArrayList<RigidBody>();
    ArrayList<Force> Forces = new ArrayList<Force>();
    ArrayList<Spring> Springs = new ArrayList<Spring>();

    public Example(int width, int height) {
        this.width = width;
        this.height = height;
        world = new Simulation(width, height);
    }

    public void initWorld() {

        for(int i = 0; i < 5; i++) {
            Bodies.add(world.addRigidBody(0.01, 10, 10, 1.0, false).
                    setCMPosition(new Vector2(40 - i*20,-i*20+250)));
        }

        for(int i = 0; i < 5; i++) {
            Bodies.add(world.addRigidBody(0.05, 20, 20, 1.0, false).
                    setCMPosition(new Vector2(-40 + i*20,50)));
        }

        Forces.add(world.addForce(new Vector2(0, 200), -1000, 300, false));

        Bodies.add(world.addRigidBody(0.01, 600, 10, 1, true));

        for(int i = 0; i < 4; i++)
            Bodies.add(world.addRigidBody(0.01, 50, 10, 1.0, false).
                    setCMPosition(new Vector2(i*100-125, -300)));

        Springs.add(world.addSpring(new Vector2(-150, -300), Bodies.get(11), 2, 30, 15));
        Springs.add(world.addSpring(Bodies.get(11), 3, Bodies.get(12), 2, 30, 15));
        Springs.add(world.addSpring(Bodies.get(12), 3, Bodies.get(13), 2, 30, 15));
        Springs.add(world.addSpring(Bodies.get(13), 3, Bodies.get(14), 2, 30, 15));
        Springs.add(world.addSpring(new Vector2(150, -300), Bodies.get(14), 3, 30, 15));

        Forces.add(world.addForce(new Vector2(0, -350), 1000, 300, false));
        Forces.add(world.addForce(new Vector2(100, -300), 1000, 300, false));

        world.enableGravity(false).enableBorder(false);
    }

    public void renderWorld() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the framebuffer

        int halfWidth = (width + 100)/2;
        int halfHeight = (height + 100)/2;

        glLineWidth(1.5f);

        glColor3f(0.0f,0.0f,0.0f);

        for(int i = 0; i < Springs.size(); i++) {

            Spring spr = Springs.get(i);
            int x1, x2, y1, y2;

            x1 = (int)spr.getAttachment1().x;
            y1 = (int)spr.getAttachment1().y;

            x2 = (int)spr.getAttachment2().x;
            y2 = (int)spr.getAttachment2().y;
            //System.out.println(x1/(double)width + " " + y1);
            glBegin(GL_LINES);
            glVertex2d(x1/(double)width*2, y1/(double)height*2);
            glVertex2d(x2/(double)width*2, y2/(double)height*2);
            glEnd();

            //g.drawLine(x1+halfWidth, -y1+halfHeight, x2+halfWidth, -y2+halfHeight);
        }

        Random r = new Random();

        float[][] colors = {{1.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 1.0f}};


        for(int i = 0; i < Bodies.size(); i++) {

            glColor3d(colors[i%colors.length][0], colors[i%colors.length][1], colors[i%colors.length][2]);

            RigidBody body = Bodies.get(i);

            glBegin(GL_QUADS);
            glVertex2d((int)body.getVertex(1).x/(double)width*2,
                    (int)body.getVertex(1).y/(double)height*2);

            glVertex2d((int)body.getVertex(2).x/(double)width*2,
                    (int)body.getVertex(2).y/(double)height*2);

            glVertex2d((int)body.getVertex(3).x/(double)width*2,
                    (int)body.getVertex(3).y/(double)height*2);

            glVertex2d((int)body.getVertex(4).x/(double)width*2,
                    (int)body.getVertex(4).y/(double)height*2);

            glEnd();

            glColor3i(0, 0, 0);

            glBegin(GL_LINES);
            glVertex2d((int)body.getVertex(1).x/(double)width*2,
                    (int)body.getVertex(1).y/(double)height*2);

            glVertex2d((int)body.getVertex(2).x/(double)width*2,
                    (int)body.getVertex(2).y/(double)height*2);

            glVertex2d((int)body.getVertex(2).x/(double)width*2,
                    (int)body.getVertex(2).y/(double)height*2);

            glVertex2d((int)body.getVertex(3).x/(double)width*2,
                    (int)body.getVertex(3).y/(double)height*2);

            glVertex2d((int)body.getVertex(3).x/(double)width*2,
                    (int)body.getVertex(3).y/(double)height*2);

            glVertex2d((int)body.getVertex(4).x/(double)width*2,
                    (int)body.getVertex(4).y/(double)height*2);

            glVertex2d((int)body.getVertex(4).x/(double)width*2,
                    (int)body.getVertex(4).y/(double)height*2);

            glVertex2d((int)body.getVertex(1).x/(double)width*2,
                    (int)body.getVertex(1).y/(double)height*2);
            glEnd();
        }

        for(int i = 0; i < Forces.size(); i++) {
            int x = (int)Forces.get(i).getPosition().x;
            int y = (int)Forces.get(i).getPosition().y;

            glBegin(GL_LINES);
            glVertex2d((x-5)/(double)width*2, y/(double)height*2);
            glVertex2d((x+5)/(double)width*2, y/(double)height*2);
            glVertex2d(x/(double)width*2, (y+5)/(double)height*2);
            glVertex2d(x/(double)width*2, (y-5)/(double)height*2);
            glEnd();
        }
    }

    public void run() {
        System.out.println("Hello LWJGL " + Version.getVersion() + "!");

        init();
        loop();

        // Free the window callbacks and destroy the window
        glfwFreeCallbacks(window);
        glfwDestroyWindow(window);

        // Terminate GLFW and free the error callback
        glfwTerminate();
        glfwSetErrorCallback(null).free();
    }

    private void init() {
        // Setup an error callback. The default implementation
        // will print the error message in System.err.
        GLFWErrorCallback.createPrint(System.err).set();

        // Initialize GLFW. Most GLFW functions will not work before doing this.
        if ( !glfwInit() )
            throw new IllegalStateException("Unable to initialize GLFW");

        // Configure GLFW
        glfwDefaultWindowHints(); // optional, the current window hints are already the default
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // the window will stay hidden after creation
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE); // the window will be resizable

        // Create the window
        window = glfwCreateWindow(width, height, "Hello World!", NULL, NULL);
        if ( window == NULL )
            throw new RuntimeException("Failed to create the GLFW window");

        // Setup a key callback. It will be called every time a key is pressed, repeated or released.
        glfwSetKeyCallback(window, (window, key, scancode, action, mods) -> {
            if ( key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE )
                glfwSetWindowShouldClose(window, true); // We will detect this in the rendering loop
        });

        // Get the thread stack and push a new frame
        try ( MemoryStack stack = stackPush() ) {
            IntBuffer pWidth = stack.mallocInt(1); // int*
            IntBuffer pHeight = stack.mallocInt(1); // int*

            // Get the window size passed to glfwCreateWindow
            glfwGetWindowSize(window, pWidth, pHeight);

            // Get the resolution of the primary monitor
            GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());

            // Center the window
            glfwSetWindowPos(
                    window,
                    (vidmode.width() - pWidth.get(0)) / 2,
                    (vidmode.height() - pHeight.get(0)) / 2
            );
        } // the stack frame is popped automatically

        // Make the OpenGL context current
        glfwMakeContextCurrent(window);
        // Enable v-sync
        glfwSwapInterval(1);

        // Make the window visible
        glfwShowWindow(window);
    }

    private void loop() {
        // This line is critical for LWJGL's interoperation with GLFW's
        // OpenGL context, or any context that is managed externally.
        // LWJGL detects the context that is current in the current thread,
        // creates the GLCapabilities instance and makes the OpenGL
        // bindings available for use.
        GL.createCapabilities();

        // Set the clear color
        glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

        double fps = 1000/60.;

        // Run the rendering loop until the user has attempted to close
        // the window or has pressed the ESCAPE key.
        while ( !glfwWindowShouldClose(window) ) {

            long startRendering=System.nanoTime();

            long durationMs;
            // now waits
//            do
//            {
//                //duration of the frame rendering in ms :
//                durationMs=TimeUnit.NANOSECONDS.toMillis(System.nanoTime()-startRendering);
//            } while (durationMs < fps);

            world.Simulate(fps/1000.);

            renderWorld();

            glfwSwapBuffers(window); // swap the color buffers

            // Poll for window events. The key callback above will only be
            // invoked during this call.
            glfwPollEvents();
        }
    }

    public static void main(String[] args) {
        Example demo = new Example(600, 800);

        demo.initWorld();;
        demo.run();
    }

}