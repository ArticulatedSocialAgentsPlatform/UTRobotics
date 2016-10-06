package nl.utwente.hmi.mwdialogue.informationstate.helper;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.JsonNodeType;
import com.fasterxml.jackson.databind.node.NullNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;

import nl.utwente.hmi.worker.MiddlewareToInformationStateWorker;
import nl.utwente.hmi.middleware.worker.Worker;
import nl.utwente.hmi.mwdialogue.ScenarioConfigLoader;
import nl.utwente.hmi.mwdialogue.ScenarioController;
import hmi.flipper.behaviourselection.template.value.Value;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultList;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Item;
import hmi.flipper.informationstate.List;
import hmi.flipper.informationstate.Record;
import hmi.flipper.informationstate.Item.Type;

public class RecordHelper {

	private static Logger logger = LoggerFactory.getLogger(RecordHelper.class.getName());
	private ObjectMapper om;

	public RecordHelper(){
		this.om = new ObjectMapper();
	}
	
	/**
	 * Converts a flipper Item (Record or Value) to equivalent JSON datastructure.
	 * @param i the item to transform
	 * @return the JsonNode that is equivalent to this Item
	 */
	public JsonNode convertISToJSON(Item i){
		if(i.getType() == Item.Type.Integer){
			return new IntNode(i.getInteger());
		} else if(i.getType() == Item.Type.Double){
			return new DoubleNode(i.getDouble());
		} else if(i.getType() == Item.Type.String){
			return new TextNode(i.getString());
		} else if(i.getType() == Item.Type.List){
			List l = i.getList();
			ArrayNode an = om.createArrayNode();
			
			for(Item innerItem : l.getFullList()){
				JsonNode innerNode = convertISToJSON(innerItem);
				an.add(innerNode);
			}
			
			return an;
		} else if(i.getType() == Item.Type.Record){
			Record r = i.getRecord();
			ObjectNode on = om.createObjectNode();
			
			for(Entry<String, Item> e : r.getItems().entrySet()){
				JsonNode innerNode = convertISToJSON(e.getValue());
				on.set(e.getKey(), innerNode);
			}
			
			return on;
		}
		
		return NullNode.getInstance();
	}
	
	/**
	 * Converts a JsonNode to equivalent flipper Item representation
	 * @param jn the JSON to convert
	 * @return the equivalent flipper Item
	 */
	public Item convertJSONToIS(JsonNode jn){
		if(jn.isDouble()){
			return new DefaultItem(jn.asDouble());
		} else if(jn.isInt()){
			return new DefaultItem(jn.asInt());
		} else if(jn.isTextual()){
			return new DefaultItem(jn.asText());
		} else if(jn.isObject()){
			ObjectNode on = (ObjectNode)jn;
			DefaultRecord rec = new DefaultRecord();
			
			Iterator<Entry<String, JsonNode>> it = on.fields();
			while(it.hasNext()){
				Entry<String, JsonNode> e = it.next();
				Item inner = convertJSONToIS(e.getValue());
				rec.set(e.getKey(), inner.getValue());
			}
						
			return new DefaultItem(rec);
		} else if(jn.isArray()){
			ArrayNode an = (ArrayNode)jn;
			DefaultList list = new DefaultList();
			
			Iterator<JsonNode> it = an.elements();
			while(it.hasNext()){
				JsonNode e = it.next();
				Item inner = convertJSONToIS(e);
				list.addItemEnd(inner);
			}

			return new DefaultItem(list);
			
		} else {
			return new DefaultItem("empty");
		}
	}
	
}
