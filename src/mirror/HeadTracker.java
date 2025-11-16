package mirror;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class HeadTracker {
	private final Process process;
	private final Thread readerThread;

	// latest head position (meters in kinect space)
	public volatile double headX = 0.0;
	public volatile double headY = 0.0;
	public volatile double headZ = 1.5; // default 1.5m in front of kinect

	public HeadTracker(String executablePath) throws IOException {
		ProcessBuilder pb = new ProcessBuilder(executablePath);
		// inherit PATh, etc. working dir dosent matter if we use full path
		pb.redirectErrorStream(true); // merge stdout and stderr
		process = pb.start();
		
		readerThread = new Thread(this::readLoop, "HeadTrack-Reader");
	    readerThread.setDaemon(true);
	    readerThread.start();
	}
	private void readLoop() {
		try (BufferedReader br = new BufferedReader(new InputStreamReader(process.getInputStream()))) {

			String line;
			while ((line = br.readLine()) != null) {
				line = line.trim();
				if (!line.startsWith("HEAD")) {
					continue;
				}

				// Expected format: HEAD x y z
				String[] parts = line.split("\\s+");
				if (parts.length < 4)
					continue;

				try {
					double x = Double.parseDouble(parts[1]);
					double y = Double.parseDouble(parts[2]);
					double z = Double.parseDouble(parts[3]);

					headX = x;
					headY = y;
					headZ = z;
				} catch (NumberFormatException e) {
					// ignore bad lines
				}
			}

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public double getHeadX() {
		return headX;
	}

	public double getHeadY() {
		return headY;
	}

	public double getHeadZ() {
		return headZ;
	}

	public void stop() {
		process.destroy();
	}
}
