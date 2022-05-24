package org.dyn4j.samples;

import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.listener.StepListener;

public class SpringBalance extends SimulationFrame {

    public SpringBalance() {
        super("SpringBalance", 256.0);

        //this.setOffsetY(-200);
    }

    // max force without spring - 3.18
    // max force with imaginary spring 0.13 K = 30;
    // TODO 00 max force with levers and spring

    private final static double LOAD_UP = Math.toRadians(30);
    private final static double LOAD_DOWN = Math.toRadians(-10);

    private final static double LOAD_SPD = Math.toRadians(0);

    private final static double LOAD_M = 0.55; // kg
    private final static double LOAD_D = 0.59; // m to the mass

    private static double SPRING_K;
    private static double SPRING_REST;



    private SimulationBody createRod(Vector2 a, Vector2 b) {
        Vector2 o = new Vector2(0.0, 0.0);
        Vector2 d = b.copy().subtract(a);
        Vector2 dn = d.copy().getNormalized().multiply(0.1);

        Vector2 al = dn.getLeftHandOrthogonalVector();
        Vector2 ar = dn.getRightHandOrthogonalVector();
        Vector2 bl = d.copy().add(al);
        Vector2 br = d.copy().add(ar);

        SimulationBody rod = new SimulationBody();
        rod.addFixture(Geometry.createPolygon(ar, al, bl, br), 0.001, 0.0, 0.0);
        rod.translate(a);
        rod.setMass(MassType.NORMAL);

        return rod;
    }



    private Spring<SimulationBody> spring;

    private SimulationBody lever23;

    private RevoluteJoint<SimulationBody> motor;


    public class Foo implements StepListener {

        @Override
        public void begin(TimeStep step, PhysicsWorld world) {
            double now = motor.getJointAngle();
            if (now > LOAD_UP) {
                motor.setMotorSpeed(-LOAD_SPD);
            } else if (now < LOAD_DOWN) {
                motor.setMotorSpeed(LOAD_SPD);
            }

        }

        @Override
        public void updatePerformed(TimeStep step, PhysicsWorld world) {

        }

        @Override
        public void postSolve(TimeStep step, PhysicsWorld world) {

            System.out.println("" + motor.getReactionTorque(step.getInverseDeltaTime()));
        }

        @Override
        public void end(TimeStep step, PhysicsWorld world) {
        }
    }


    private void springSetup(double... ml) {
        if (ml[0] != 0) throw new IllegalArgumentException();
        SPRING_REST = ml[1];
        for (int i = 2; i < ml.length; i += 2) {
            double m = ml[i];
            double f = 9.8 * m;
            double l = ml[i + 1];
            double k = f / (l - SPRING_REST);
            SPRING_K = k;
            //System.out.println("k=" + k);
        }
    }


    @Override
    protected void initializeWorld() {
        //weight, s, m, l
        //0, 43,57,111
        //538,47,58,111
        //1972,79,74,136
        //4747,146,109,189
    // world.getSettings().setStepFrequency(0.6);

        springSetup(0, .043, 0.538, .047, 1.972, .079, 4.747, .146); // small


        this.world.addStepListener(new Foo());

        SimulationBody ground = new SimulationBody();
        ground.addFixture(Geometry.createRectangle(0.2, 0.5));
        ground.setMass(MassType.INFINITE);
        world.addBody(ground);


        SimulationBody load = new SimulationBody();
        double density = 2.0;
        double load_y = LOAD_M / (2 * LOAD_D * density);
        load.addFixture(Geometry.createRectangle(2 * LOAD_D, load_y), density, 0.0, 0.0);
        load.translate(new Vector2(-LOAD_D, 0.0));
        load.setMass(MassType.NORMAL);
        world.addBody(load);

        double l = 0.1;
        Vector2 p3 = new Vector2(0.0, 0.0);
        Vector2 p2 = new Vector2(l, 0.0);



        lever23 = createRod(p3, p2);
        world.addBody(lever23);



        // provides the motion
        motor = new RevoluteJoint<SimulationBody>(ground, load, new Vector2(0.0, 0.0));
        motor.setMotorEnabled(true);
        motor.setMotorSpeed(-LOAD_SPD);
        motor.setMaximumMotorTorque(10000.0);

        world.addJoint(motor);

        // links the lever to the load
        Joint<SimulationBody> loadToLever23 = new WeldJoint<SimulationBody>(load, lever23, p3);
        world.addJoint(loadToLever23);

        double fl = SPRING_REST + (LOAD_M * LOAD_D * 9.8) / (SPRING_K * l);
        spring = new Spring<SimulationBody>(ground, lever23, new Vector2(l, -fl), p2);

        spring.setRestDistance(SPRING_REST);
        spring.setK(SPRING_K);

        spring.setDampingRatio(0.3);
        world.addJoint(spring);
    }

    /**
     * Entry point for the example application.
     * @param args command line arguments
     */
    public static void main(String[] args) {
        SpringBalance simulation = new SpringBalance();
        simulation.run();
    }
}
