package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class WriteToCSV extends SubsystemBase {
	String m_Filename = "/home/lvuser/match_data.csv";
	final boolean m_Append = false;
	BufferedWriter m_File;

	public WriteToCSV() {
		String path = "/home/lvuser/";
		String filename = "match_data-a";
		System.out.println("Initializing log file.");
		
		try {
			//Check if file exists
			File tempFile = new File(String.format("%s%s.csv", path, filename));

			if(true == tempFile.exists())
			{
				filename = "match_data-b";
			}

			m_Filename = String.format("%s%s.csv", path, filename);

			// Open File
			FileWriter fstream = new FileWriter(m_Filename, m_Append);
			m_File = new BufferedWriter(fstream);

			//Write the header
			writeHeader();

			System.out.println("Opened log file at " + m_Filename);
		} catch (Exception e) {
			System.out.println("Failed to open log file. " + m_Filename);
		}
	}

	@Override
    public void periodic() {
        try {
			m_File.flush();
		} catch (Exception e) {
			//Cannot flush
		}
    }

	private void writeHeader() {
        String stringToWrite = "ID, Time, TriggerMotorSet, BeltMotorSet, RpmSet, CurrentRpm, LauncherReady, DistanceToTarget, AngleToTargetDeg, IsTargeted\n";
        writeToFile(stringToWrite);
    }

	// Writes a string to the data file.
	// Returns true if successful, false otherwise
	public boolean writeToFile(String stringToWrite) {
		boolean retval = true;

		try {
			// Write string to file
			m_File.append(stringToWrite);
		} catch (Exception e) {
			// Failed to write
			retval = false;
		}

		return retval;
	}
}
