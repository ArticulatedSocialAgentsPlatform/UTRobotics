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
package nl.utwente.hmi.yarp.worker;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import yarp.Bottle;

/**
 * A simple abstract implementation of a worker thread. This thread has an internal queue which is filled with Bottles by a YARP listener. 
 * Each Bottle is then processed by the processBottle method, which can be implemented by subclasses to fit their specific needs
 * @author davisond
 *
 */
public abstract class AbstractWorker implements Worker {

	private BlockingQueue<Bottle> queue = null;

	private boolean running = true;
	
	/**
	 * Create a new AbstractWorker object. This provides the basics required for recieving bottles into the processing queue
	 */
	public AbstractWorker(){
		this.queue = new LinkedBlockingQueue<Bottle>();
	}
	
	/**
	 * Adds a certain bottle to the processing queue of this worker
	 */
	@Override
	public void addBottleToQueue(Bottle b) {
		queue.add(b);
	}

	/**
	 * This method is constantly reading the processing queue. 
	 * If a new bottle comes in, it calls the processBottle() method, which can do the actual processing as required for the specific worker.
	 */
	@Override
	public void run() {
		while(running){
			try {
				Bottle b = queue.take();
				processBottle(b);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	/**
	 * Does the required processing on the bottle. 
	 * Each worker is responsible for implementing this method as required.
	 * @param b the incoming bottle
	 */
	abstract protected void processBottle(Bottle b);

}
