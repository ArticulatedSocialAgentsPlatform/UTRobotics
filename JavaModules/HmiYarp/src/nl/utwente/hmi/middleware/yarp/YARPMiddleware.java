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
