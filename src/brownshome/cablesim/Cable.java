package brownshome.cablesim;

import brownshome.vecmath.*;

import java.util.Arrays;
import java.util.concurrent.Semaphore;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * The cable represents a collection of points spaced evenly along a cable.
 */
public class Cable {
	private static int numberOfThreads = 20;

	private final MVec3 satPosition;
	private final MVec3 satVelocity;
	private final double satMass;

	private final DoubleUnaryOperator brakingForce;

	private MVec3[] positions;
	private MVec3[] velocities;

	private double[] masses;

	/** This is the tension in the link between two of the points */
	private double[] tensions;

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
	public Cable(Vec3 satPosition, Vec3 satVelocity, double satMass, double endMass, double pointDistance, double linearDensity, UnaryOperator<Vec3> forcePerUnitLength, Vec3 deploymentVelocity, DoubleUnaryOperator brakingForce) {
		this.satPosition = satPosition.copy();
		this.satVelocity = satVelocity.copy();

		this.brakingForce = brakingForce;

		positions = new MVec3[] { satPosition.copy() };
		velocities = new MVec3[] { deploymentVelocity.copy() };
		velocities[0].add(satVelocity);

		// End mass
		masses = new double[] { endMass };
		this.satMass = satMass;

		tensions = new double[0];

		this.interPointLength = pointDistance;
		this.linearDensity = linearDensity;
		this.forcePerUnitLength = forcePerUnitLength;

		correctLength(positions.length - 1);
	}

	public void step(double timestep) {
		time += timestep;

		addPoint(timestep);

		applyForces(timestep);

		applyBrakingForce(timestep);

		correctVelocities(timestep);
		movePoints(timestep);

		correctLength(positions.length - 1);
	}

	private void addPoint(double timestep) {
		Vec3 position = positions[positions.length - 1];

		MVec3 difference = position.copy();
		difference.subtract(satPosition);

		double l = difference.length();

		if(l > interPointLength) {
			// Add point

			positions = Arrays.copyOf(positions, positions.length + 1);

			MVec3 addedPos = position.copy();

			addedPos.scale(1.0 - interPointLength / l);
			addedPos.scaleAdd(satPosition, interPointLength / l);

			positions[positions.length - 1] = addedPos;

			velocities = Arrays.copyOf(velocities, velocities.length + 1);

			// Start at 0
			velocities[velocities.length - 1] = satVelocity.copy();

			masses = Arrays.copyOf(masses, masses.length + 1);

			masses[masses.length - 1] = linearDensity * interPointLength;

			tensions = Arrays.copyOf(tensions, tensions.length + 1);

			correctVelocities(timestep);
		}
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

		satVelocity.scaleAdd(forcePerUnitLength.apply(satPosition), timestep);

		for(int i = 0; i < velocities.length; i++) {
			velocities[i].scaleAdd(forcePerUnitLength.apply(positions[i]), timestep);
		}
	}

	private void applyBrakingForce(double timestep) {
		double force = brakingForce.applyAsDouble(getLength());

		if(force == 0)
			return;

		Vec3 positionOfSatEnd = positions[positions.length - 1];
		MVec3 velocityOfSatEnd = velocities[velocities.length - 1];

		MVec3 distanceFromSat = positionOfSatEnd.copy();
		distanceFromSat.subtract(satPosition);

		MVec3 relativeVelocity = velocityOfSatEnd.copy();
		relativeVelocity.subtract(satVelocity);

		MVec3 appliedForce;

		if(distanceFromSat.lengthSq() < 0.001) {
			if(relativeVelocity.lengthSq() < 0.001)
				return;

			appliedForce = relativeVelocity;
		} else if(relativeVelocity.dot(distanceFromSat) > 0) {
			appliedForce = distanceFromSat;
		} else {
			return;
		}

		appliedForce.normalize();
		appliedForce.scale(-force);

		velocityOfSatEnd.scaleAdd(appliedForce, timestep / masses[positions.length - 1]);
		satVelocity.scaleAdd(appliedForce, -timestep / satMass);
	}

	/** Ensure that velocities are never such that they would cause the cable to stretch */
	private void correctVelocities(double timestep) {
		if(positions.length == 1) {
			return;
		}

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
			MVec3 vec = positions[i + 1].copy();
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

		for(int i = 0; i < numLinks; i++) {
			tensions[i] = tensions[i] * 0.9999 + J[i] * 0.0001 / timestep;
		}

		velocities[0].scaleAdd(R[0], J[0] / masses[0]);

		for(int i = 1; i < numPoints - 1; i++) {
			velocities[i].scaleAdd(R[i - 1], -J[i - 1] / masses[i]);
			velocities[i].scaleAdd(R[i], J[i] / masses[i]);
		}

		velocities[numPoints - 1].scaleAdd(R[numLinks - 1], -J[numLinks - 1] / masses[numPoints - 1]);
	}

	private void movePoints(double timestep) {
		satPosition.scaleAdd(satVelocity, timestep);

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

	public double getLength() {
		return positions.length * interPointLength;
	}

	public Vec3 getSatPosition() {
		return satPosition;
	}
}
