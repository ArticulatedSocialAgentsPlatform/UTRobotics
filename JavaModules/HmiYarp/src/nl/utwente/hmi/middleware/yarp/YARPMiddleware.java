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
package nl.utwente.hmi.middleware.yarp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;

import yarp.Bottle;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.MiddlewareListener;
import nl.utwente.hmi.yarp.helper.YARPHelper;
import nl.utwente.hmi.yarp.listener.YARPPortListener;
import nl.utwente.hmi.yarp.worker.AbstractWorker;
import nl.utwente.hmi.yarp.writer.YARPPortWriter;

/**
 * Implementation for the YARP middleware.
 * This class implements a YARPPortWriter for sending Bottles and a YARPPortListener (and associated Worker) for recieving Bottles.
 * The Bottles are converted via XML to Data
 * @author davisond
 *
 */
public class YARPMiddleware extends AbstractWorker implements Middleware {
	private static Logger logger = LoggerFactory.getLogger(YARPMiddleware.class.getName());

	private String iPortName;
	private String oPortName;

	private YARPPortWriter portWriter;
	private YARPPortListener portListener;

	private YARPHelper yarpHelper;

	private Set<MiddlewareListener> listeners;

	/**
	 * Create this Middleware using the specified ports
	 * @param yarpEPortName the external port name on which we want to listen for data
	 * @param iPortName the internal port name on which we want to receive data (this will be connected to the yarpEPortName)
	 * @param oPortName the output port name on which we send our data
	 */
	public YARPMiddleware(String iPortName, String oPortName){
		this.iPortName = iPortName;
		this.oPortName = oPortName;
		
		this.listeners = Collections.synchronizedSet(new HashSet<MiddlewareListener>());
		this.yarpHelper = new YARPHelper();
		
		this.portWriter = new YARPPortWriter(oPortName);
		portWriter.init();
		
		new Thread(this).start();
		portListener = new YARPPortListener(iPortName);
		portListener.addWorker(this);
		
		new Thread(portListener).start();
	}
	
	@Override
	public void sendData(JsonNode jn) {
		if(jn != null){
			logger.debug("Sending data: {}", jn.toString());
			Bottle b = yarpHelper.convertJSONToBottle(jn);
			logger.debug("Converted to bottle: {}", b.toString());
			portWriter.writeBottle(b);
		}
	}
	
	@Override
	public void sendDataRaw(String str){
		portWriter.writeBottle(new Bottle(str));
	}

	@Override
	protected void processBottle(Bottle b) {
		logger.debug("Got bottle: {}", b.toString());
		JsonNode jn = yarpHelper.convertBottleToJSON(b);
		logger.debug("Transformed to JSON: {}", jn.toString());
		
		if(jn != null){
			for(MiddlewareListener ml : listeners){
				ml.receiveData(jn);
			}
		}
	}
	
	@Override
	public void addListener(MiddlewareListener ml) {
		this.listeners.add(ml);
	}
	
}
