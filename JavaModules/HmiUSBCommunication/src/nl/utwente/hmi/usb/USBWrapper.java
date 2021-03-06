/*******************************************************************************
 * Copyright (C) 2009-2020 Human Media Interaction, University of Twente, the Netherlands
 *
 * This file is part of the Articulated Social Agents Platform BML realizer (ASAPRealizer).
 *
 * ASAPRealizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ASAPRealizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ASAPRealizer.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
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
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import gnu.io.CommPortIdentifier;
import gnu.io.NRSerialPort;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;



/**
 * This basic class wraps a USB (or actually, a serial) connection, providing a convenient way to send and receive strings from a connected USB devide, such as an Arduino
 * Implementing classes must override the processData(String) function, which is called each time new data arrives from the serial port.
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

	private BufferedReader input;

	private BufferedWriter output;

	protected SerialPort serialPort;

	public USBWrapper(String[] comPorts){
		init(comPorts);
	}
	
	private void init(String[] comPorts){
		//Set<String> availablePorts = NRSerialPort.getAvailableSerialPorts();
		//logger.debug("Searching for COM ports {} in available set of ports {}", Arrays.toString(comPorts), Arrays.toString(availablePorts.toArray()));
		
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
			//logger.error("Could not find COM port {} in available set of ports {}", Arrays.toString(comPorts), Arrays.toString(availablePorts.toArray()));
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
