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
				masses[i] = linearDensity * interPointLength / 2;
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
		correctVelocities(timestep);
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
	private void correctVelocities(double timestep) {
		// For each pair of points correct the velocity. Then average the results.
		// Treat each point as if it was split in two half mass points, except for the end points
		// We do this as a Jaccobian so it can be made parallel later
		double[] overallChangeInVelocity = new double[tensions.length];

		double error = 0;
		for(int iter = 0; iter < 10000; iter++) {
			error = singleVelocityPass(overallChangeInVelocity);
		}

		// Uncomment for error
		// System.out.println(error);

		// Save the tension values
		for(int i = 0; i < overallChangeInVelocity.length; i++) {
			// DV * massOfPoint = force
			// massOfPoint = linearDensity * interPointLength

			tensions[i] = overallChangeInVelocity[i] * linearDensity * interPointLength;
		}
	}

	/** This returns the overall error in ms-1 and does a single pass over the cable. */
	private double singleVelocityPass(double[] overallChangeInVelocity) {
		double error = 0;

		// For pair i - 1 and i

		for(int i = 1; i < velocities.length; i++) {
			// Find Dv . Dp
			// tensileDeltaV = ( Dv . Dp )

			// The vector from P1 to P2
			MVec3 deltaPosition = new MVec3(positions[i]);
			deltaPosition.subtract(positions[i - 1]);
			deltaPosition.normalize();

			// +ve means tension, -ve means compression
			double requiredChangeInVelocity = deltaPosition.dot(velocities[i]) - deltaPosition.dot(velocities[i - 1]);
			error += Math.abs(requiredChangeInVelocity);

			overallChangeInVelocity[i - 1] += requiredChangeInVelocity;

			/*

			At every point a change in velocity of V is needed, the change in velocity is shared between the objects
			1 and 2 with masses M1 and M2 by object 1 moving M2/(M1+M2) and object 2 moving M1/(M1+M2). We compute
			1/(M1+M2)

			Every object is averaged for each link it touches, so that every mass that is not the end mass appears half as
			large.

			*/

			boolean isStart = i == 1;
			boolean isEnd = i == velocities.length - 1;

			double m1 = isStart ? masses[i - 1] : masses[i - 1] / 2;
			double m2 = isEnd ? masses[i] : masses[i] / 2;

			double s1 = requiredChangeInVelocity * m2 / (m1 + m2) * (isStart ? 1 : 0.5 /* Average if not the end */);
			double s2 = -requiredChangeInVelocity * m1 / (m1 + m2) * (isEnd ? 1 : 0.5 /* Average if not the end */);

			if(m1 == Double.POSITIVE_INFINITY) {
				s1 = 0;
				s2 = -requiredChangeInVelocity * (isEnd ? 1 : 0.5 /* Average if not the end */);
			} else if(m2 == Double.POSITIVE_INFINITY) {
				s1 = requiredChangeInVelocity * (isStart ? 1 : 0.5 /* Average if not the end */);
				s2 = 0;
			}

			velocities[i - 1].scaleAdd(deltaPosition, s1);
			velocities[i].scaleAdd(deltaPosition, s2);
		}
		return error;
	}

	private void movePoints(double timestep) {
		for(int i = 0; i < positions.length; i++) {
			positions[i].scaleAdd(velocities[i], timestep);
		}
	}

	public double calculateEnergy() {
		double sum = 0;

		for(int i = 1; i < positions.length; i++) {
			sum += masses[i] * (0.5 * velocities[i].lengthSq()/* + 10 * positions[i].y()*/);
		}

		return sum;
	}

	/** Shorten or lengthen the cable to its correct length */
	private void correctLength() {
		// Pin the middle of the cable
		int middle = positions.length / 2;

		// Iterate from the middle to the +ve end
		Vec3 previous = positions[middle];
		for(int i = middle + 1; i < positions.length; i++) {
			MVec3 current = positions[i];
			current.subtract(previous);
			current.scale(interPointLength / current.length());
			current.add(previous);

			previous = current;
		}

		// Iterate from the middle to the -ve end
		previous = positions[middle];
		for(int i = middle - 1; i >= 0; i--) {
			MVec3 current = positions[i];
			current.subtract(previous);
			current.scale(interPointLength / current.length());
			current.add(previous);

			previous = current;
		}
	}
}
