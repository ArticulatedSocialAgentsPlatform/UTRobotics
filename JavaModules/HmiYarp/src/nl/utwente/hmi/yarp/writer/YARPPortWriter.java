package nl.utwente.hmi.yarp.writer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import yarp.Bottle;
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
	private Port port;


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
    	port = new Port();
    	boolean openPortSuccess = port.open(iPortName);
    	if(openPortSuccess){
    		logger.info("Registering port [{}]... Done", iPortName);
    	} else {
    		logger.error("Registering port [{}]... Failed", iPortName);
    	}
    	
    	boolean connectingSuccess = true;
    	//should we connect it to an external port..?
    	if(openPortSuccess && ePortName != null && !"".equals(ePortName)){
    		connectingSuccess = Network.connect(iPortName, ePortName);
    		if(connectingSuccess){
        		logger.info("Connecting internal port [{}] to external port [{}]....Done", iPortName, ePortName);
    		} else {
        		logger.warn("Connecting internal port [{}] to external port [{}]....Failed", iPortName, ePortName);
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
			logger.info("Sending bottle on port [{}]: {}",iPortName,b.toString());
			return port.write(b);
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
