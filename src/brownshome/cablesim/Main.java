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

		cable = new Cable(1000, 5, 1,
				// Position
				l -> new MVec3(l, 0, 0),

				// Velocity
				l -> new MVec3(0, 0, 0),

				// Force
				p -> new MVec3(0, -9.81, 0));
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(1000, 1000);
	}

	private int K = 0;

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);

		Graphics2D g2 = (Graphics2D) g;

		g.clearRect(0, 0, getWidth(), getHeight());

		double minX, minY, maxX, maxY;

		MVec3[] positions = cable.getPositions();

		int N = 1000;
		for(int i = 0; i < N; i++)
			cable.step(16.0 / 1e3 / N);

		System.out.println(K++ + " E: " + cable.calculateEnergy());

		minX = minY = -6;
		maxX = maxY = 6;

		double maxTension = 0.0;
		for(int i = 0; i < cable.getTensions().length; i++) {
			maxTension = Math.max(maxTension, cable.getTensions()[i]);
		}

		int drawN = 1;

		for(int i = drawN; i < positions.length; i += drawN) {
			((Graphics2D) g).setStroke(new BasicStroke(3));
			g.setColor(new Color((float) Math.max(Math.min(cable.getTensions()[i - 1], 1.0), 0.0), 0.0f, (float) Math.max(cable.getTensions()[i - 1] / maxTension, 0.0)));

			Line2D.Double line = new Line2D.Double(
					scale(positions[i - drawN].x(), minX, maxX) * getWidth(),
					(1 - scale(positions[i - drawN].y(), minY, maxY)) * getHeight(),
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
