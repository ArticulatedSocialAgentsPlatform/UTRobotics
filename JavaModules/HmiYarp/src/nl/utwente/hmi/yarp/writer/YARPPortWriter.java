package nl.utwente.hmi.yarp.writer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import yarp.Bottle;
import yarp.BufferedPortBottle;
import yarp.Network;
import yarp.Port;

/**
 * Simple wrapper class for sending bottles over a YARP port. Note that this class does not automatically connect the internal port to an external port.
 * @author davisond
 *
 */
public class YARPPortWriter {
	private static Logger logger = LoggerFactory.getLogger(YARPPortWriter.class.getName());

	/** The internal portname on which we send bottles */
	private String iPortName;

	/** The posisble external port name on which we want to push our bottles.. this might be empty */
	private String ePortName;

	/** The actual instance of the port on which we send */
	private BufferedPortBottle port;


	/**
	 * Create a new port writer instance, which can send to the port iPortName
	 * @param iPortName the name of the internal port we want to write to
	 */
	public YARPPortWriter(String iPortName){
		this.iPortName = iPortName;
	}

	/**
	 * Create a new port writer instance, which can send to the port iPortName, try to connect it automatically to the optionally supplied external port ePortName
	 * @param iPortName the name of the internal port we want to write to
	 * @param ePortName the name of the external port
	 */
	public YARPPortWriter(String iPortName, String ePortName){
		this.iPortName = iPortName;
		this.ePortName = ePortName;
	}
	
	/**
	 * Init the network and create the port
	 * @return
	 */
	public boolean init(){
    	
    	//create the internal output port
    	port = new BufferedPortBottle();
    	boolean openPortSuccess = port.open(iPortName);
    	if(openPortSuccess){
    		logger.info("Registering buffered ouput port [{}]... Done", iPortName);
    	} else {
    		logger.error("Registering buffered output port [{}]... Failed", iPortName);
    	}
    	
    	boolean connectingSuccess = true;
    	//should we connect it to an external port..?
    	if(openPortSuccess && ePortName != null && !"".equals(ePortName)){
    		connectingSuccess = Network.connect(iPortName, ePortName);
    		if(connectingSuccess){
        		logger.info("Connecting internal output port [{}] to external input port [{}]....Done", iPortName, ePortName);
    		} else {
        		logger.warn("Connecting internal output port [{}] to external input port [{}]....Failed", iPortName, ePortName);
    		}
    	}
    	
    	//how successful were we? :)
    	return openPortSuccess && connectingSuccess;
	}
	
	/**
	 * Writes a bottle to the port
	 * @param b the bottle
	 * @return true if success, false if failure
	 */
	public boolean writeBottle(Bottle b){
		if(port != null){
			logger.debug("Sending bottle on buffered output port [{}]: {}",iPortName,b.toString());
			
			//get a buffered bottle from the port
			Bottle bufferedBottle = port.prepare();
			
			//make sure it is empty (sometimes the port can recycle a bottle)
			bufferedBottle.clear();
			
			//add the contents of the bottle we actually want to send
			bufferedBottle.append(b);

			//tell the port to write the bottle (somehow this should work, by reference to the original object... or something.. in the original implementation they use pointers)
			//writeStrict tells YARP to never drop a bottle, but instead place them in writing queue
			port.writeStrict();
			return true;
		} else {
			return false;
		}
	}

	public String getIPortName() {
		return iPortName;
	}

	public String getEPortName() {
		return ePortName;
	}
	
	
}
