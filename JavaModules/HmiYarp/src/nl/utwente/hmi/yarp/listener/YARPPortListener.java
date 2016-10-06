package nl.utwente.hmi.yarp.listener;

import java.util.ArrayList;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import yarp.Bottle;
import yarp.Network;
import yarp.Port;
import yarp.Value;
import nl.utwente.hmi.yarp.worker.Worker;

/**
 * Lightweight thread which simply listens for incoming bottles and then passes them on to a worker object to handle whatever is necessary
 * @author davisond
 *
 */
public class YARPPortListener implements Runnable {
	private static Logger logger = LoggerFactory.getLogger(YARPPortListener.class.getName());
	
	/** The internal portname on which we listen to bottles */
	private String iPortName;

	/** The actual instance of the port on which we listen */
	private Port port;
	
	/** The worker object which will be called as soon as we receive a bottle */
	private ArrayList<Worker> workers;

	public YARPPortListener(String yarpIPort){
		this.iPortName = yarpIPort;
		workers = new ArrayList<Worker>();
	}
	
	/**
	 * Add a worker thread to this listener, each time a bottle is recieved it will be sent to all registered workers
	 * @param w the worker thread
	 */
	public void addWorker(Worker w){
		this.workers.add(w);
	}

	@Override
	public void run() {
    	
    	//create the internal input port
		
    	port = new Port();
    	if(port.open(iPortName)){
    		logger.info("Registering port [{}]... Done", iPortName);
    	} else {
    		logger.error("Registering port [{}]... Failed", iPortName);
    	}

		logger.info("Starting to listen for bottles...");
    	
		//listen for new bottles, the read() function is blocking until a bottle is received
		while(true){
		    Bottle bot = new Bottle();
		    port.read(bot);
		    //logger.debug("Got bottle on port [{}]: {}",iPortName,bot.toString());
		    for(Worker w : workers){
		    	w.addBottleToQueue(bot);
		    }
		}
	}
	
}
