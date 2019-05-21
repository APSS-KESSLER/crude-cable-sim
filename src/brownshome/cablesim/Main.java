package brownshome.cablesim;

import brownshome.vecmath.*;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import javax.swing.*;

public class Main extends JPanel {
	public static void main(String[] args) {
		double length = 100;
		double density = 1e-3;

		int points = 1000;
		double timestep = 1e-6;

		Vec2 deploymentAngle = new IVec2(0.0, 0.0);
		double deploymentSpeed = 10;

		double endMass = 0.05;
		double satMass = 1.30;

		double friction = 0.0;
		double brakingForce = 0.01;

		for(int i = 0; i < args.length; i++) {
			switch(args[i]) {
				case "-l":
				case "--length":
					length = Double.parseDouble(args[i++]);
					break;
				case "-d":
				case "--linear-density":
					density = Double.parseDouble(args[i++]);
					break;
				case "-t":
				case "--timestep":
					timestep = Double.parseDouble(args[i++]);
					break;
				case "-p":
				case "--points":
					points = Integer.parseInt(args[i++]);
					break;
				case "-a":
				case "--deployment-angle":
					deploymentAngle = new IVec2(Double.parseDouble(args[i++]), Double.parseDouble(args[i++]));
					break;
				case "-s":
				case "--deployment-speed":
					length = Double.parseDouble(args[i++]);
					break;
				case "-w":
				case "--end-mass":
					endMass = Double.parseDouble(args[i++]);
					break;
				case "-W":
				case "--sat-mass":
					satMass = Double.parseDouble(args[i++]);
					break;
				case "-b":
				case "--braking-force":
					brakingForce = Double.parseDouble(args[i++]);
					break;
				case "-f":
				case "--friction":
					friction = Double.parseDouble(args[i++]);
					break;
			}
		}

		Rot3 xRotation = IRot3.fromAxisAngle(new IVec3(0, 0, 1), deploymentAngle.x());
		Rot3 zRotation = IRot3.fromAxisAngle(new IVec3(1, 0, 0), deploymentAngle.y());

		MVec3 deploymentDirection = new MVec3(0, -1, 0);
		xRotation.rotate(deploymentDirection);
		zRotation.rotate(deploymentDirection);
		deploymentDirection.scale(deploymentSpeed);

		Main main = new Main(length, density, points, timestep, deploymentDirection, endMass, satMass, friction, brakingForce);

		SwingUtilities.invokeLater(main::startApplication);
	}

	private void startApplication() {
		JFrame frame = new JFrame();
		frame.getContentPane().add(this);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setResizable(false);
		frame.setVisible(true);
	}

	private final Cable cable;

	private final double L;
	private final double u = 3.986e14;
	private final double timestep;

	Main(double length, double density, int points, double timestep, Vec3 deploymentDirection, double endMass, double satMass, double friction, double brakingForce) {
		super(true);

		L = length;
		this.timestep = timestep;

		Timer timer = new Timer(16, event -> {
			repaint();
		});
		timer.start();

		cable = new Cable(new IVec3(0, 6.771e6, 0), new IVec3(Math.sqrt(u / 6.771e6), 0, 0), satMass, endMass, L / points, density, p -> {
					MVec3 vec = new MVec3(p);

					vec.normalize();
					vec.scale(-u / p.lengthSq());

					return vec;
				}, deploymentDirection, l -> {
					if(l < L) {
						return friction;
					} else {
						return friction + brakingForce;
					}
				});
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(800, 800);
	}

	private int K = 0;

	@Override
	public synchronized void paintComponent(Graphics g) {
		super.paintComponent(g);

		Graphics2D g2 = (Graphics2D) g;

		g.clearRect(0, 0, getWidth(), getHeight());

		double minX, minY, maxX, maxY;

		for(int i = 0; i < 2000; i++) {
			cable.step(timestep);
		}

		MVec3[] positions = cable.getPositions();

		minX = minY = Double.POSITIVE_INFINITY;
		maxX = maxY = Double.NEGATIVE_INFINITY;

		for(var vec : cable.getPositions()) {
			minX = Math.min(minX, vec.x());
			maxX = Math.max(maxX, vec.x());
			minY = Math.min(minY, vec.y());
			maxY = Math.max(maxY, vec.y());
		}

		double centerX = (minX + maxX) / 2;
		double centerY = (minY + maxY) / 2;

		double range = L * 1.2;

		minX = centerX - range / 2;
		minY = centerY - range / 2;
		maxX = centerX + range / 2;
		maxY = centerY + range / 2;

		double maxTension = 0.0;
		for(int i = 0; i < cable.getTensions().length; i++) {
			maxTension = Math.max(maxTension, cable.getTensions()[i]);
		}

		int drawN = 1;

		MVec2 center = new MVec2(-centerX, -centerY);
		center.normalize();
		center.scale(0.8);

		g2.setStroke(new BasicStroke(3));

		g2.draw(new Ellipse2D.Double((0.5 + center.x() / 2) * getWidth(), (0.5 - center.y() / 2) * getHeight(), 50, 50));

		for(int i = drawN; i < positions.length; i += drawN) {
			g.setColor(new Color((float) Math.max(Math.min(cable.getTensions()[i - 1], 1.0), 0.0), (float) Math.max(cable.getTensions()[i - 1] / maxTension, 0.0), 0));

			Line2D.Double line = new Line2D.Double(
					scale(positions[i - drawN].x(), minX, maxX) * getWidth(),
					(1 - scale(positions[i - drawN].y(), minY, maxY)) * getHeight(),
					scale(positions[i].x(), minX, maxX) * getWidth(),
					(1 - scale(positions[i].y(), minY, maxY)) * getHeight());

			g2.draw(line);
		}

		g2.setColor(Color.BLACK);

		Rectangle2D.Double sat = new Rectangle2D.Double(
				scale(cable.getSatPosition().x(), minX, maxX) * getWidth() - 25,
				(1 - scale(cable.getSatPosition().y(), minY, maxY)) * getHeight() - 25,
				50,
				50);

		g2.draw(sat);

		g2.drawString(String.format("Height: %.3fkm", (cable.getPositions()[cable.getPositions().length / 2].length() - 6.371e6) * 1e-3), 50, 50);
		g2.drawString(String.format("Time: %.3fs", cable.getTime()), 50, 70);
		g2.drawString(String.format("Velocity: %.3fms-1", cable.getVelocities()[cable.getVelocities().length / 2].length()), 50, 90);
		g2.drawString(String.format("Length: %.3fm", cable.getLength()), 50, 110);
		// g2.drawString(String.format("Max Tension: %.3fN", maxTension), 50, 130);

		repaint();
	}

	private double scale(double v, double min, double max) {
		return (v - min) / (max - min);
	}
}
