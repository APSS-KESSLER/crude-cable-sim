package brownshome.cablesim;

import brownshome.vecmath.*;

import java.util.concurrent.Semaphore;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * The cable represents a collection of points spaced evenly along a cable.
 */
public class Cable {
	private static int numberOfThreads = 20;

	private final MVec3[] positions;
	private final MVec3[] velocities;

	private final double[] masses;
	private final double[] lengthControlled;

	/** This is the tension in the link between two of the points */
	private final double[] tensions;

	private final double interPointLength;
	private final UnaryOperator<Vec3> forcePerUnitLength;

	private final double linearDensity;

	private double time = 0;

	/**
	 * @param points The number of points on the cable
	 * @param length The length of the cable in m
	 * @param linearDensity The density of the cable in Kg/m
	 * @param initialPosition A function that returns the position of the cable given L
	 * @param initialVelocity A function that returns the velocity of the cable given L
	 * @param forcePerUnitLength A function that returns the force placed on the cable per unit length for a given position
	 */
	public Cable(int points, double length, double linearDensity, DoubleFunction<MVec3> initialPosition, DoubleFunction<MVec3> initialVelocity, UnaryOperator<Vec3> forcePerUnitLength) {
		assert points > 2;

		positions = new MVec3[points];
		velocities = new MVec3[points];
		masses = new double[points];
		lengthControlled = new double[points];

		tensions = new double[points - 1];

		this.interPointLength = length / (points - 1);
		this.linearDensity = linearDensity;
		this.forcePerUnitLength = forcePerUnitLength;

		for(int i = 0; i < points; i++) {
			double l = i * interPointLength;

			// The mass of a point is related to the amount of material it holds
			// The lengthControlled variable is used to calculate the amount of force to apply.
			if(i == 0) {
				lengthControlled[i] = interPointLength / 2;
				masses[i] = Double.POSITIVE_INFINITY; //linearDensity * interPointLength / 2;
			} else if(i == points - 1) {
				lengthControlled[i] = interPointLength / 2;
				masses[i] = linearDensity * interPointLength / 2;
			} else {
				lengthControlled[i] = interPointLength;
				masses[i] = linearDensity * interPointLength;
			}

			positions[i] = initialPosition.apply(l);
			velocities[i] = initialVelocity.apply(l);
		}

		correctLength();
	}

	public void step(double timestep) {
		time += timestep;

		applyForces(timestep);

		correctVelocities();
		movePoints(timestep);
		correctLength();
	}

	public double getTime() {
		return time;
	}

	public double[] getTensions() {
		return tensions;
	}

	public MVec3[] getPositions() {
		return positions;
	}

	public MVec3[] getVelocities() {
		return velocities;
	}

	private void applyForces(double timestep) {
		// dV = forcePerLength * lengthPerPoint / massPerPoint * timestep

		for(int i = 0; i < velocities.length; i++) {
			velocities[i].scaleAdd(forcePerUnitLength.apply(positions[i]), lengthControlled[i] / masses[i] * timestep);
		}
	}

	/** Ensure that velocities are never such that they would cause the cable to stretch */
	private void correctVelocities() {
		// Ri is the normalized vector from the Pi to Pi+1

		// Mi and Ni, are the resistance to change of the i. M is to change from the negative side. N is to change
		// from the positive side.

		// Vi is the error velocity of the n'th link
		// Ii is the attractive impulse of Ri
		// Ji is the attractive impulse, taking into account all other attractive impulses.

		int numLinks = tensions.length;
		int numPoints = positions.length;

		Vec3[] R = new Vec3[numLinks];
		for(int i = 0; i < numLinks; i++) {
			MVec3 vec = new MVec3(positions[i + 1]);
			vec.subtract(positions[i]);
			vec.normalize();
			R[i] = vec;
		}

		// M0 is NaN as there is no link into the 0th vertex
		double[] M = new double[numPoints];

		// The last point has no outgoing link
		M[numPoints - 1] = masses[numPoints - 1];

		for(int i = numPoints - 2; i > 0; i--) {
			// Mi * ChangeInVi = Ii
			// We split the applied Ii into component with the next R and the component perp to it.

			double withR2 = R[i - 1].dot(R[i]);
			withR2 *= withR2;

			double perpR2 = 1 - withR2;

			double motion = withR2 / (masses[i] + M[i + 1]) + perpR2 / masses[i];
			M[i] = 1 / motion;
		}

		// The last point is NaN as it has no outgoing link
		double[] N = new double[numPoints];

		N[0] = masses[0];

		for(int i = 1; i < numPoints - 1; i++) {
			// Ni * ChangeInVi = Ii
			// We split the applied Ii into component with the next R and the component perp to it.

			double withR2 = R[i - 1].dot(R[i]);
			withR2 *= withR2;

			double perpR2 = 1 - withR2;

			double motion = withR2 / (masses[i] + N[i - 1]) + perpR2 / masses[i];
			N[i] = 1 / motion;
		}

		// +ve means that the points are drifting apart.
		double[] V = new double[numLinks];

		for(int i = 0; i < numLinks; i++) {
			V[i] = R[i].dot(velocities[i + 1]) - R[i].dot(velocities[i]);
		}

		double[] I = new double[numLinks];

		for(int i = 0; i < numLinks; i++) {
			I[i] = V[i] / (1.0 / N[i] + 1.0 / M[i + 1]);
		}

		// J = I + contributions from other I
		double[] J = I.clone();

		double extraImpulse = 0.0;
		for(int i = 1; i < numLinks; i++) {
			// J += contributions from I[< i]
			// Adjust by cos * (I[i-1] * (1.0 - m[i]/M[i]))

			extraImpulse = (I[i - 1] + extraImpulse) * (1.0 - masses[i] / M[i]) / R[i - 1].dot(R[i]);
			J[i] += extraImpulse;
		}

		extraImpulse = 0.0;
		for(int i = numLinks - 2; i >= 0; i--) {
			// J += contributions from I[> i]

			extraImpulse = (I[i + 1] + extraImpulse) * (1.0 - masses[i + 1] / N[i + 1]) / R[i].dot(R[i + 1]);
			J[i] += extraImpulse;
		}

		velocities[0].scaleAdd(R[0], J[0] / masses[0]);

		for(int i = 1; i < numPoints - 1; i++) {
			velocities[i].scaleAdd(R[i - 1], -J[i - 1] / masses[i]);
			velocities[i].scaleAdd(R[i], J[i] / masses[i]);
		}

		velocities[numPoints - 1].scaleAdd(R[numLinks - 1], -J[numLinks - 1] / masses[numPoints - 1]);
	}

	private void movePoints(double timestep) {
		for(int i = 0; i < positions.length; i++) {
			positions[i].scaleAdd(velocities[i], timestep);
		}
	}

	public double calculateEnergy() {
		double sum = 0;

		for(int i = 1; i < positions.length; i++) {
			sum += masses[i] * (0.5 * velocities[i].lengthSq() + 9.81 * positions[i].y());
		}

		return sum;
	}

	private void correctLength() {
		// Pin the middle if in doubt
		correctLength(positions.length / 2);
	}

	/** Shorten or lengthen the cable to its correct length */
	private void correctLength(int pin) {
		// Iterate from the pin to the +ve end
		Vec3 previous = positions[pin];
		for(int i = pin + 1; i < positions.length; i++) {
			MVec3 current = positions[i];
			current.subtract(previous);
			current.scale(interPointLength / current.length());
			current.add(previous);

			previous = current;
		}

		// Iterate from the middle to the -ve end
		previous = positions[pin];
		for(int i = pin - 1; i >= 0; i--) {
			MVec3 current = positions[i];
			current.subtract(previous);
			current.scale(interPointLength / current.length());
			current.add(previous);

			previous = current;
		}
	}
}
