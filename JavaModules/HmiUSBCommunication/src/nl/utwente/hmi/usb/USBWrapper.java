package nl.utwente.hmi.usb;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;


/**
 * This basic class wraps a USB (or actually, a serial) connection, providing a convenient way to send and receive strings from a connected USB devide, such as an Arduino
 * Implementing classes must override the processData(String) function, which is called each time new data arrives from the serial port.
 * 
 * TODO: currently only works on Windows machines!!! possibly move to a different RXTX fork for better compatibilty with other operating systems: https://github.com/NeuronRobotics/nrjavaserial
 *
 * @author davisond
 *
 */
public abstract class USBWrapper implements SerialPortEventListener {

	/**
	 * Thrown when an exception occurs while processing data from the USB connection
	 * @author davisond
	 *
	 */
	public class DataProcessingException extends Exception {

		private static final long serialVersionUID = 4235790504829305068L;

		public DataProcessingException(String msg) {
			super(msg);
		}

	}
	
	private static Logger logger = LoggerFactory.getLogger(USBWrapper.class.getName());

	private static final int TIME_OUT = 2000;
	private static final int DATA_RATE = 9600;

	private SerialPort serialPort;

	private BufferedReader input;

	private BufferedWriter output;

	public USBWrapper(String[] comPorts){
		init(comPorts);
	}
	
	private void init(String[] comPorts){
		logger.debug("Searching for COM ports: {}", Arrays.toString(comPorts));
		
		// Setting up serial connection
		CommPortIdentifier portId = null;
		Enumeration<CommPortIdentifier> portEnum = CommPortIdentifier.getPortIdentifiers();

		// First, Find an instance of serial port as set in PORT_NAMES.
		while (portEnum.hasMoreElements()) {
			CommPortIdentifier currPortId = portEnum.nextElement();
			for (String portName : comPorts) {
				if (currPortId.getName().equals(portName)) {
					logger.debug("Found COM port: {}", currPortId.getName());
					portId = currPortId;
					break;
				}
			}
		}
		
		//did we succeed in getting the correct COM?
		if (portId == null) {
			logger.error("Could not find COM port: [{}].", Arrays.toString(comPorts));
			System.exit(-1);
		}

		//now make the actual connection and add ourselves as listener for incoming serial data
		try {
			logger.debug("Opening serial connection to COM port {}", portId.getName());
			serialPort = (SerialPort) portId.open(this.getClass().getName(), TIME_OUT);
			serialPort.setSerialPortParams(DATA_RATE, SerialPort.DATABITS_8, SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);

			// open the streams
			input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
			output = new BufferedWriter(new OutputStreamWriter(serialPort.getOutputStream()));
			
			serialPort.addEventListener(this);
			serialPort.notifyOnDataAvailable(true);
		} catch (Exception e) {
			logger.error("Exception while setting up the USB Serial connection on COM port {}: {}", serialPort.getName(), e.getMessage());
			e.printStackTrace();
		}
	}
	
	
	@Override
	public synchronized void serialEvent(SerialPortEvent oEvent) {
		if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
			try {
				String inputLine = null;
				if (input.ready()) {
					inputLine = input.readLine();
					logger.debug("Got data from USB: {}", inputLine);
					
					//catch any debugging information we might get from the board
					if(inputLine.startsWith("Debug")){
						logger.debug("Got debug message from ARDUINO: {}", inputLine.substring(7, inputLine.length()));
						return;
					}
					
					//now leave it to the implementing object to actually process the data
					processData(inputLine);
				}

			} catch(DataProcessingException dpe){
				logger.error("Error while processing data: {}", dpe.getMessage());
				dpe.printStackTrace();
			}
			catch (Exception e) {
				logger.error("Error while receiving data from serial port: {}", e.getMessage());
				e.printStackTrace();
			}
		} else {
			logger.warn("Got other SerialPortEvent: {}", oEvent);
		}
	}
	
	/**
	 * Attempts to write and flush the provided string to the serial port. The string is automatically appended by a newline character.
	 * @param data the data do write (note that it will be automatically appended by a newline character)
	 */
	public void sendData(String data){
		try {
			output.write(data);
			output.newLine();
			output.flush();
		} catch (IOException e) {
			logger.error("Failed to write data to serial port {}: [{}]", serialPort.getName(), data);			
			e.printStackTrace();
		}
	}
	
	/**
	 * Does the actual processing of the incoming data
	 * @param data the data to be processed
	 * @throws DataProcessingException when something is wrong with the data
	 */
	protected abstract void processData(String data) throws DataProcessingException;
}
