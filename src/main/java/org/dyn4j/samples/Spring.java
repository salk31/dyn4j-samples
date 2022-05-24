package org.dyn4j.samples;

import org.dyn4j.dynamics.PhysicsBody;
import org.dyn4j.dynamics.joint.DistanceJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;

public class Spring<T extends PhysicsBody> extends DistanceJoint<T> {

    public Spring(T body1, T body2, Vector2 anchor1, Vector2 anchor2) {
        super(body1, body2, anchor1, anchor2);
    }

    public void setK(double k) {
        double lm = this.getReducedMass();
        //double nf = Geometry.TWO_PI * frequency;
        //k = mass * naturalFrequency * naturalFrequency;
        // nf = sqrt(k / mass)
        // f = sqrt(k / mass) / (2*PI)
        setFrequency(Math.sqrt(k / lm) / Geometry.TWO_PI);
    }
}
