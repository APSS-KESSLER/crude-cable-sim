package brownshome.cablesim;

import brownshome.vecmath.MVec3;

import java.awt.*;
import java.awt.geom.Line2D;

import javax.swing.*;

public class Main extends JPanel {
	public static void main(String[] args) {
		SwingUtilities.invokeLater(Main::startApplication);
	}

	private static void startApplication() {
		JFrame frame = new JFrame();
		frame.getContentPane().add(new Main());
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setResizable(false);
		frame.setVisible(true);
	}

	private final Cable cable;

	Main() {
		super(true);

		repaint();

		cable = new Cable(100, 5, 1,
				// Position
				l -> new MVec3(l - 2.5, 0, 0),

				// Velocity
				l -> new MVec3(0, (l - 2.5) * (l - 2.5) * (l - 2.5), 0),

				// Force
				p -> new MVec3(0, 0, 0));
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(1000, 1000);
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);

		Graphics2D g2 = (Graphics2D) g;

		g.clearRect(0, 0, getWidth(), getHeight());

		double minX, minY, maxX, maxY;

		MVec3[] positions = cable.getPositions();

		cable.step(16.0/10000.0);

		minX = minY = -6;
		maxX = maxY = 6;

		double maxTension = 0.0;
		for(int i = 0; i < cable.getTensions().length; i++) {
			maxTension = Math.max(maxTension, cable.getTensions()[i]);
		}

		for(int i = 1; i < positions.length; i++) {
			((Graphics2D) g).setStroke(new BasicStroke(3));
			g.setColor(new Color((float) Math.max(Math.min(cable.getTensions()[i - 1], 1.0), 0.0), 0.0f, (float) Math.max(cable.getTensions()[i - 1] / maxTension, 0.0)));

			Line2D.Double line = new Line2D.Double(
					scale(positions[i - 1].x(), minX, maxX) * getWidth(),
					(1 - scale(positions[i - 1].y(), minY, maxY)) * getHeight(),
					scale(positions[i].x(), minX, maxX) * getWidth(),
					(1 - scale(positions[i].y(), minY, maxY)) * getHeight());

			g2.draw(line);
		}

		g2.drawString(String.format("Energy: %.3f", cable.calculateEnergy()), 50, 50);
		g2.drawString(String.format("Time: %.3fs", cable.getTime()), 50, 70);

		repaint();
	}

	private double scale(double v, double min, double max) {
		return (v - min) / (max - min);
	}
}