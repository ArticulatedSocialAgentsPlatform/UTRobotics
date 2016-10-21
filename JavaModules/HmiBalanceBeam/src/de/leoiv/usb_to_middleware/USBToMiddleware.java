package de.leoiv.usb_to_middleware;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Properties;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import de.leoiv.usb_to_middleware.USBToMiddleware.DataProcessingException;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.array;
import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.object;

import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.GenericMiddlewareLoader;

/**
 * The following class will read the output from the arduino, convert it to JSON,
 * and send it to the middleware so other modules can use the data.
 * 
 * TODO: possibly move to a different RXTX fork for better compatibilty with other operating systems: https://github.com/NeuronRobotics/nrjavaserial
 * TODO: it would be cool if the USB communication would also implement a Middleware, thus making bridging much easier :-)
 * 
 * @author Leonard Hövelmann, leonard.hoevelmann@posteo.de
 * @author Daniel Davison
 *
 */
public class USBToMiddleware implements SerialPortEventListener {
	public class DataProcessingException extends Exception {

		private static final long serialVersionUID = 4235790504829305068L;

		public DataProcessingException(String msg) {
			super(msg);
		}

	}

	private static Logger logger = LoggerFactory.getLogger(USBToMiddleware.class.getName());

	private static final int TIME_OUT = 2000;
	private static final int DATA_RATE = 9600;

	private JsonNode previousData = null;

	private SerialPort serialPort;

	private BufferedReader input;

	private Middleware mw;

	private String[] comPortNames;
	
	public USBToMiddleware(Middleware mw, String[] comPortNames){
		this.mw = mw;
		this.comPortNames = comPortNames;
	}

	/**
	 * Init this USB converter to listen for messages on one of the specified COM ports and send data on to the middleware
	 * @param comPortNames a list of COM port names applicable to this PC (COM1, COM2, ..)
	 * @param mw the middleware on which to send messages
	 */
	public void init() {
		logger.debug("Searching for COM ports: {}", Arrays.toString(comPortNames));
		
		// Setting up serial connection
		CommPortIdentifier portId = null;
		Enumeration<CommPortIdentifier> portEnum = CommPortIdentifier.getPortIdentifiers();

		// First, Find an instance of serial port as set in PORT_NAMES.
		while (portEnum.hasMoreElements()) {
			CommPortIdentifier currPortId = portEnum.nextElement();
			for (String portName : comPortNames) {
				if (currPortId.getName().equals(portName)) {
					logger.debug("Found COM port: {}", currPortId.getName());
					portId = currPortId;
					break;
				}
			}
		}
		
		//did we succeed in getting the correct COM?
		if (portId == null) {
			logger.error("Could not find COM port.");
			return;
		}

		//now make the actual connection and add ourselves as listener for incoming serial data
		try {
			logger.debug("Opening serial connection to COM port {}", portId.getName());
			serialPort = (SerialPort) portId.open(this.getClass().getName(), TIME_OUT);
			serialPort.setSerialPortParams(DATA_RATE, SerialPort.DATABITS_8, SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);

			// open the streams
			input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
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
					
					JsonNode data = processSensorData(inputLine);
					
					//only send if data has actually changed
					//TODO: should keep rfid data separate from beam data
					if(!data.equals(previousData)){
						logger.info("Sending balance sensor data to middleware: {}", data.toString());
						mw.sendData(data);
					}
					
					previousData = data;
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
			logger.debug("Got other SerialPortEvent: {}", oEvent);
		}
	}
	
	/**
	 * Processes a line of data from the sensors and produces a JsonNode that can be sent to the middleware
	 * The data follows the following format: 
	 * - starting with "ID" if it is an RFID event, followed by the actual rfid tag id: ["ID"][rfidtag]
	 * - or containing the state of the balance beam as a sequence of 9 characters, matching the REGEX "[01]{2,2}[123][0LMH]{6,6}": [blockleft][blockright][tilt][pot1]..[pot6], where blockleft and blockright is [0|1] for [placed|notplaced], tilt is [1|2|3] for [left|center|right] and pot is [0|L|M|H] for [none|light|medium|heavy]
	 * @param data a line of incoming sensor data
	 * @return a JSonNode that contains the data suitable for sending over the middleware
	 * @throws DataProcessingException if data does not match the expected format
	 */
	private JsonNode processSensorData(String data) throws DataProcessingException {
		if (data.indexOf("ID") >= 0) {
			logger.debug("Processing RFID: {}", data);
			//format: "IDxxxxxxx"
			String rfid = data.substring(2, data.length());
			
			//build JSON: {sensordata:{rfid:{id:"xxxxx"}}}
			JsonNode json = object("sensordata", object()
				                .with("rfid",
					                  object("id",rfid)
				                )
							).end();
			
			return json;
		} else if(data.length() == 9 && data.matches("[01]{2,2}[123][0LMH]{6,6}")) {
			logger.debug("Processing balance sensor data: {}", data);
			boolean reedLeft = data.charAt(0) == '1';
			boolean reedRight = data.charAt(1) == '1';
			
			int potMeter = Character.getNumericValue((data.charAt(2)));
			String tilt = potMeter == 1 ? "left" : potMeter == 2 ? "center" : "right";
			
			//TODO: it might be nice to send also a list of individual values to the middleware instead of only a string of positions
			String pots = data.substring(3, 9);
			
			//now build the JSON: {sensordata:{balancebeam:{blocksplaced:{left:"TRUE|FALSE", right:"TRUE|FALSE"},tilt:"left|center|right",pots:"xxxxxx"}}}
			JsonNode json = object("sensordata", object()
								.with("balancebeam", object()
									.with("blocksplaced", object()
										.with("left", Boolean.toString(reedLeft).toUpperCase())
										.with("right", Boolean.toString(reedRight).toUpperCase())
									)
									.with("tilt", tilt)
									.with("pots", pots)
								)
							).end();
			
			return json;
		} else {
			logger.error("Got malformed line of data: {}", data);
			throw new DataProcessingException("Unable to process data: "+data);
		}
	}

}
