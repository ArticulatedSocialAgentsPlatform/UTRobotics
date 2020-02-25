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
package nl.utwente.hmi.worker;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;

import nl.utwente.hmi.middleware.Middleware;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Record;
import nl.utwente.hmi.mwdialogue.informationstate.Observer;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;

public class InformationStateToMiddlewareWorker implements Observer {
	private static Logger logger = LoggerFactory.getLogger(InformationStateToMiddlewareWorker.class.getName());

	private RecordHelper rh;
	private Middleware middleware;

	public InformationStateToMiddlewareWorker(Middleware middleware){
		this.middleware = middleware;

		this.rh = new RecordHelper();
	}
	
	@Override
	public void hasChanged(Record record) {
		if(record.getItems().size() > 0){
			logger.debug("IS has changed: {}",record.toString());
			
			JsonNode jn = rh.convertISToJSON(new DefaultItem(record));
			logger.debug("Transforming to JSON: {}", jn.toString());
	
			logger.debug("Sending to middleware ");
			middleware.sendData(jn);
		} else {
			logger.warn("The specified record is empty!");
		}
	}

}
