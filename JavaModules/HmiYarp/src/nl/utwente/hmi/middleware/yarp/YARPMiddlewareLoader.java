package nl.utwente.hmi.middleware.yarp;

import java.util.Properties;
import java.util.Map.Entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.MiddlewareLoader;

public class YARPMiddlewareLoader implements MiddlewareLoader {
	private static Logger logger = LoggerFactory.getLogger(YARPMiddlewareLoader.class.getName());

	/**
	 * This loads the STOMPMiddleware instance
	 */
	@Override
	public Middleware loadMiddleware(Properties ps) {
		Middleware m = null;
		String iPortName = "";
		String oPortName = "";
		
		for(Entry<Object, Object> entry : ps.entrySet()){
            System.out.println("propkey:"+(String)entry.getKey());
            System.out.println("propval:"+(String)entry.getValue());
            if(((String)entry.getKey()).equals("iPortName")){
				iPortName = (String)entry.getValue();
			}
            if(((String)entry.getKey()).equals("oPortName")){
				oPortName = (String)entry.getValue();
			}
		}
		
		if(iPortName.equals("") || oPortName.equals("")){
			logger.error("Could not load the YARPMiddleware, need at least properties: iPortName and oPortName");
		} else {
			m = new YARPMiddleware(iPortName, oPortName);
		}
		
		return m;
	}

}
