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
package nl.utwente.hmi.yarp.helper;

import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonArrayFormatVisitor;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.JsonNodeType;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;

import yarp.Bottle;
import yarp.Network;
import yarp.Port;
import yarp.Value;

/**
 * A collection of random helper functions for processing and/or generating YARP Bottles and Values.
 * Just to make your life a bit easier :)
 * 
 * The functions of this class assume the bottles follow the following hierarchy convention: [key1] [value1] [value2] [[key2] [value21] [value22]] [value3]
 * @author davisond
 *
 */
public class YARPHelper {
	private static Logger logger = LoggerFactory.getLogger(YARPHelper.class.getName());

	private String newLineChar = System.getProperty("line.separator");

	private ObjectMapper om;
	
	public YARPHelper(){
		om = new ObjectMapper();
	}
	
	/**
	 * Recursively transforms a bottle datastructure to JSON datastructure. 
	 * Bottle is a nestes structure of key-values, where a value can be another bottle, or a leaf node (both can be mixed in the same list of values...)
	 * Can handle the following simple (nested) structures:
	 * - Takes bottles like (key value) and transforms them to {key:value}
	 * - Bottles like (value .. value) to [value, .. , value]
	 * 
	 * Bottles with key and multiple values are transformed as best as possible to sensible JSON, using the following rules:
	 * - If value is an inner list without key, such as "addpersons ((name daniel) (name piet))", we add an additional key "anonymouslist_x", like so: {addpersons:{anonymouslist_1:[{name:daniel},{name:piet}]}}
	 * - If value is an inner key-values pair, we try to add the key-values directly to the parent node: add (person (name daniel)) becomes {add:{person:{name:daniel}}}. However, if the parent already contains this key, we create a list for this key that contains both nested values: add (person (name daniel)) (person (name piet)) becomes {add:{person:[{name:piet},{name:daniel}]}}
	 * - If value is a leaf node (and there is more than one) we add them as "v_1 .. v_x" to retain direct access. For instance (phonenrs 0123456789 0987654321) is translated to {phonenrs:{{v_1:0123456789},{v_2:0987654321}}}
	 * @param b the bottle to transform
	 * @return a JsonNode that represents the bottle datastructure
	 */
	public JsonNode convertBottleToJSON(Bottle b){
		//empty bottle
		if(b == null || b.isNull() || b.size() == 0 || b.get(0).isNull()){
			return null;
		}

		//is this a leaf?
		if(b.size() == 1 && (b.get(0).isInt() || b.get(0).isDouble() || b.get(0).isString())){
			return convertValueToJSON(b.get(0));
		}

		//not a leaf, so this must be a list of nested stuff
		String key = "";
		boolean keyPresent = false;
		
		//check if this list has a specified key, or its just a normal anonymous list
		if(b.get(0).isString()){
			key = b.get(0).asString();
			keyPresent = true;
			b = b.tail();
		}

		//if this is a key-value pair, we can simply add it
		if(keyPresent && b.size() == 1 && (b.get(0).isInt() || b.get(0).isDouble() || b.get(0).isString())){
			ObjectNode on = om.createObjectNode();
			
			JsonNode inner = convertValueToJSON(b.get(0));
			on.set(key, inner);
			return on;
		}
		
		//if not, we need to start doing some funky stuff..
		//first, lets collect a list of the inner values
		ArrayNode innerValues = om.createArrayNode();
		for(int i = 0; i < b.size(); i++){
			innerValues.add(convertValueToJSON(b.get(i)));
		}
		
		ObjectNode root = om.createObjectNode();
		ObjectNode inner = om.createObjectNode();

		ArrayNode innerArray = om.createArrayNode();
		ArrayNode leaves = om.createArrayNode();
		
		//different types of inner nodes need to be treated differently
		for(JsonNode jn : innerValues){
			if(jn.isObject()){
				//inner object nodes should be added directly to this node with their key-values
				//this allows for better/easier access by other data processors
				//example bottle person (name daniel) (id 5): instead of {person:[{name:daniel},{id:5}]} (would access using something like person.get(1).get(name))
				//we would expect something like: {person:{{name:daniel},{id:5}}} (would access by person.get(name))
				ObjectNode innerON = (ObjectNode)jn;
				Iterator<Entry<String,JsonNode>> innerIt = innerON.fields();
				while(innerIt.hasNext()){
					Entry<String,JsonNode> e = innerIt.next();
					
					//if we already know this key, we create an inner array so we don't loose information
					//an example: addpersons (person (name d)) (person (name t))
					//this will become addpersons:{person:[{name:d},{name:t}]}
					if(inner.get(e.getKey()) != null){
						if(inner.get(e.getKey()).getNodeType() == JsonNodeType.ARRAY){
							ArrayNode tmpArr = (ArrayNode)inner.get(e.getKey());
							tmpArr.add(e.getValue());
							inner.set(e.getKey(), tmpArr);
						} else {
							ArrayNode tmpArr = om.createArrayNode();
							tmpArr.add(inner.get(e.getKey()));
							tmpArr.add(e.getValue());
							inner.set(e.getKey(), tmpArr);
						}
					} else {
						//key is not yet known: we can safely add the inner JsonNode to the parent using it's key and value
						inner.set(e.getKey(), e.getValue());
					}
				}
			} else if(jn.isArray()){
				//inner (anonymous) lists need to be treaded differently
				//example: addpersons ((name d) (name t))
				//should become: addpersons:{anonymouslist_1:[{name:d},{name:t}]}
				innerArray.add(jn);
			} else {
				//if we have more than one leaf, we need to preserve the order of the values for direct access
				//example: (phonenrs 0123456789 0987654321)
				//should translate to: {phonenrs:{{v_1:0123456789},{v_2:0987654321}}}
				leaves.add(jn);
			}
		}

		if(innerArray.size() > 0){
			int i = 1;
			for(JsonNode jn : innerArray){
				inner.set("anonymouslist_"+i, jn);
				i++;
			}
		}

		if(leaves.size() > 0){
			int i = 1;
			for(JsonNode jn : leaves){
				inner.set("v_"+i, jn);
				i++;
			}
		}
		
		if(keyPresent){
			root.set(key, inner);
			return root;
		} else {
			return inner;
		}
		
	}
	
	/**
	 * Helper method for transforming a YARP value to JSON
	 * @param v the value (could be a leaf node or another nested bottle)
	 * @return the JSON node corresponding to this value
	 */
	private JsonNode convertValueToJSON(Value v){
		//is this a leaf?
		if(v.isInt()){
			return new IntNode(v.asInt());
		} else if(v.isDouble()){
			return new DoubleNode(v.asDouble());
		} else if(v.isString()){
			return new TextNode(v.asString());
		} else if(v.isList()){
			return convertBottleToJSON(v.asList());
		} else {
			logger.error("Value node of type [{}] is unknown: {}", v.getType() ,v.toString());
			return null;
		}
	}
	
	/**
	 * Transforms a JSON node back to a Bottle datatstructure. 
	 * Relatively straightforward, this reverses the rules as specified by "transformBottleToJSON()".
	 * @param jn The JSON node to transform
	 * @return the Bottle that represents the JSON
	 */
	public Bottle convertJSONToBottle(JsonNode jn){
		Bottle ret = convertJSONToBottle(jn, new Bottle());
		//this is just one bottle with one inner record, so we strip away the outside container
		if(ret.size() == 1){
			return ret.get(0).asList();
		} else {
			return ret;
		}
	}
	

	/**
	 * Transforms and adds a JSON node to the specified bottle
	 * Relatively straightforward, this reverses the rules as specified by "transformBottleToJSON()".
	 * @param jn the JSON node to ransform
	 * @param b the bottle to add it to
	 * @return the bottle b with added jn
	 */
	private Bottle convertJSONToBottle(JsonNode jn, Bottle b){
		if(jn.isDouble()){
			b.add(new Value(jn.asDouble()));
		} else if(jn.isInt()){
			b.add(new Value(jn.asInt()));
		} else if(jn.isTextual()){
			b.add(new Value(jn.asText()));
		} else if(jn.isObject()){
			ObjectNode on = (ObjectNode)jn;
			Iterator<Entry<String, JsonNode>> it = on.fields();
			while(it.hasNext()){
				Entry<String, JsonNode> e = it.next();
				if(e.getKey().startsWith("anonymouslist_")){
					convertJSONToBottle(e.getValue(), b);
				} else if(e.getKey().startsWith("v_")) {
					convertJSONToBottle(e.getValue(), b);
				} else {
					if(e.getValue().isArray()){
						for(JsonNode innerJN : e.getValue()){
							Bottle innerBottle = b.addList();
							innerBottle.add(e.getKey());
							convertJSONToBottle(innerJN, innerBottle);
						}
					} else {
						Bottle innerBottle = b.addList();
						innerBottle.add(e.getKey());
						convertJSONToBottle(e.getValue(), innerBottle);
					}
				}
			}
		} else if(jn.isArray()){
			ArrayNode an = (ArrayNode)jn;
			Iterator<JsonNode> it = an.elements();
			Bottle innerBottle = b.addList();
			while(it.hasNext()){
				JsonNode e = it.next();
				convertJSONToBottle(e, innerBottle);
			}
		}
		return b;
	}
	
	
	/**
	 * This function recursively searches the specified Bottle for a specific key. It then returns the whole bottle (including key and values) which contains the key. Only the first encountered instance of a key is returned!
	 * Bottles are a hierarchical list of nested key-values, where the key (String) is always the first item in the list and the values can be one or multiple String/int/floats etc. The values may also be another nested Bottle.
	 * So the structure must be similar as follows: [key1] [value1] [value2] [[key2] [value21] [value22]] [value3]
	 * <br /><br />
	 * Example input: "hello hi 1 2 (test a b c) (ja 3 4) hi ik (testing AB CD (EF 22) 42) jaja (another (stuff hello))"
	 * Searching for key "testing" will return bottle: "testing AB CD (EF 22) 42"
	 * @param b the nested bottle which to search
	 * @param key the key to search for
	 * @return null if not found, or the Bottle which contains the key and values
	 */
	public Bottle searchKey(Bottle b, String key){
		//if b is empty/non-existant just return null
		if(b == null || b.isNull() || b.size() == 0 || b.get(0).isNull()){
			return null;
		}
		
		//if this is the bottle we are looking for (hurray!), return the whole bottle containing the key and all values
		if(b.size() > 0 && b.get(0).isString() && b.get(0).asString().equals(key)){
			return b;
		} 
		
		//otherwise, try to recurse into the hierarchy to perhaps find the key on a deeper level
		if(b.size() > 0){
			int i = 0;
			Bottle bTmp = null;
			
			//never give up, don't stop till we found it or have exhausted our list
			while(bTmp == null && i < b.size()){
				if(b.get(i).isList()){
					//let's go deeper!
					bTmp = searchKey(b.get(i).asList(), key);
				}
				i++;
			}
			
			//return the first bottle found, or null
			return bTmp;
		}
		
		//we found nothing :(
		return null;
	}
	
	/**
	 * A more elaborate search function, similar to searchKey(). This continues searching recursively (depth-first) through an entire bottle, and returns a List of ALL found bottles with the specified key.
	 * Bottles are a hierarchical list of nested key-values, where the key (String) is always the first item in the list and the values can be one or multiple String/int/floats etc. The values may also be another nested Bottle.
	 * So the structure must be similar as follows: [key1] [value1] [value2] [[key2] [value21] [value22]] [value3]
	 * <br /><br />
	 * Example input: (testing (nested key with values) (something (nested other key (nested key) with different (possibly deeper) values)) (nested stuff)) (nested toplevel)<br />
	 * Example output: [nested key with values, nested other key (nested key) with different (possibly deeper) values, nested stuff, nested toplevel]
	 * @param b the bottle to search for
	 * @param key the key to search for
	 * @return a List<Bottle> of all found bottles. List will be empty if no nested bottles are found
	 */
	public List<Bottle> searchAllKeys(Bottle b, String key){

		//stores all the bottles we have found
		ArrayList<Bottle> found = new ArrayList<Bottle>();
		
		//if b is empty/non-existant just return empty list
		if(b == null || b.isNull() || b.size() == 0 || b.get(0).isNull()){
			return found;
		}
		
		//if this is the bottle we are looking for (hurray!), return the whole bottle containing the key and all values
		if(b.size() > 0 && b.get(0).isString() && b.get(0).asString().equals(key)){
			found.add(b);
			return found;
		} 
		
		//otherwise, try to recurse into the hierarchy to perhaps find the key on a deeper level
		if(b.size() > 0){
			int i = 0;
			
			//never give up, don't stop till we found it or have exhausted our list
			while(i < b.size()){
				if(b.get(i).isList()){
					//let's go deeper!
					found.addAll(searchAllKeys(b.get(i).asList(), key));
				}
				i++;
			}
		}
		
		//return all the bottles
		return found;
	}
	
	
}
