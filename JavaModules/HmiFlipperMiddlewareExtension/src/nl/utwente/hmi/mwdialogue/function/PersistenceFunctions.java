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
package nl.utwente.hmi.mwdialogue.function;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Serializable;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import nl.utwente.hmi.middleware.helpers.JSONHelper;
import nl.utwente.hmi.mwdialogue.ScenarioController;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;
import nl.utwente.hmi.mwdialogue.persistence.PersistentRecord;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Item;
import hmi.flipper.informationstate.Record;

//TODO: could extend this to save full history in json file, only retrieving newest timestamp, or preferred timestamp/version
public class PersistenceFunctions implements FunctionClass {
    private static Logger logger = LoggerFactory.getLogger(PersistenceFunctions.class.getName());

    private static final String DATA_DIR = "storage/";
    private static final String DATA_EXTENSION = ".json";
    
	private Record is;
	private RecordHelper rh;
	private ObjectMapper om;

	public PersistenceFunctions(Record is){
		this.is = is;
		om = new ObjectMapper();
		rh = new RecordHelper();
	}

	/**
	 * Store a certain subsection of the IS for future use, as a string JSON representation.
	 * Duplicate IDs will be overwritten
	 * 
	 * @param params This function expects 2 String arguments: first an ID, used to retrieve this data and secondly a path (WITHOUT THE $-SIGN) to store
	 */
	public void storeRecord(Object... params){
		if(params instanceof String[] && params.length == 2){
			String[] p = (String[])params;
			String id = p[0];
			String path = "$"+p[1];
			
			try {
				if(is.getTypeOfPath(path) == Item.Type.Record){
					//retrieve the record/item we want to store
					Item i = new DefaultItem(is.getRecord(path));
					
					//convert it to JSON
					JsonNode jsonRec = rh.convertISToJSON(i);
					
					//select where we want to store it
					FileOutputStream f_out = new FileOutputStream(DATA_DIR+id+DATA_EXTENSION);
					PrintWriter pw = new PrintWriter(f_out);

					//add some metadata so we can restore it to the same location
					ObjectNode store = om.createObjectNode();
					store.put("path", path);
					store.put("timestamp", System.currentTimeMillis());
					store.put("id", id);
					store.set("content", jsonRec);
					
					//now save it :)
					logger.info("Storing record [{}] at [{}]: {}", new String[]{id, path, store.toString()});
					pw.print(store.toString());
					pw.flush();
					
					pw.close();
					f_out.close();
				} else {
					logger.warn("Unable to store data {}, only support data of type record", path);
				}
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Load a subsection of the IS from storage. This record will be placed back in the IS, overwriting it if applicable
	 * @param params one argument: the ID
	 */
	public void loadRecord(Object... params){
		if(params instanceof String[] && params.length == 1){

			String[] p = (String[])params;
			String id = p[0];

			String fileName = DATA_DIR+id+DATA_EXTENSION;
			
			try {
				
				logger.info("Loading data from file {}", fileName);
				File f = new File(fileName);
				
				//does it exist?
				if(!f.isFile()){
					logger.warn("File doesn't exist: {}", fileName);
					DefaultRecord r = new DefaultRecord();
					r.set("loadedsuccess","FALSE");
					is.set("$persistence", r);
				} else {
					//load data from file and convert to JSON
					String data = new String(Files.readAllBytes(Paths.get(f.getPath())));
					JsonNode jn = om.readTree(data);
					
					//check if the data holds what we need
					if(jn.path("path").isTextual() && jn.path("content").isObject()){
						String path = jn.get("path").asText();
						logger.info("Restoring data to IS: {}", path);
						
						//convert JSON to record
						Item item = rh.convertJSONToIS(jn.get("content"));
						
						//now restore the data back to the IS
						is.set(path, item);
						
						//let any waiting templates know the loading was succesful
						DefaultRecord r = new DefaultRecord();
						r.set("loadedsuccess","TRUE");
						is.set("$persistence", r);
					} else {
						logger.warn("Data {} missing 'path' and/or 'content': {}", fileName, data);
						DefaultRecord r = new DefaultRecord();
						r.set("loadedsuccess","FALSE");
						is.set("$persistence", r);
					}
				}
			} catch (Exception e) {
				//this means that we didnt find this record, unfortunately
				logger.warn("Something went wrong while loading data: {}", fileName);
				DefaultRecord r = new DefaultRecord();
				r.set("loadedsuccess","FALSE");
				is.set("$persistence", r);
				e.printStackTrace();
			}

		}
	}
	
}
