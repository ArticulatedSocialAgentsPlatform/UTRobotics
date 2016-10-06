package nl.utwente.hmi.yarp.worker;

import yarp.Bottle;

/**
 * A Worker class can take care of a YARP bottle message
 * @author davisond
 *
 */
public interface Worker extends Runnable {

	/**
	 * Place bottle in processingQueue of the worker
	 * @param b
	 */
	public void addBottleToQueue(Bottle b);
	
}
