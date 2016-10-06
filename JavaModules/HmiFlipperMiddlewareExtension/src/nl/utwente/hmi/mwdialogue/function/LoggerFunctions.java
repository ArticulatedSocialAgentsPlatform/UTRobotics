package nl.utwente.hmi.mwdialogue.function;

import java.util.Arrays;

import nl.utwente.hmi.mwdialogue.ScenarioController;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LoggerFunctions {
    private static Logger logger = LoggerFactory.getLogger("FlipperLogger");

	
	public void logMessage(Object... params){
		//this seems ugly.. but it was the only way I could get it to work with the reflection invoke...
		if(params instanceof String[]){
			String[] par = (String[])params;
			if(params.length >= 2){
				String level = par[0];
				String msg = par[1];
				String[] ps = Arrays.copyOfRange(par, 2, par.length);
				if("trace".equals(level)){
					logger.trace(msg, ps);
				} else if("debug".equals(level)){
					logger.debug(msg, ps);
				} else if("warn".equals(level)){
					logger.warn(msg, ps);
				} else if("error".equals(level)){
					logger.error(msg, ps);
				} else {
					logger.info(msg, ps);
				}
			} else if(params.length == 1){
				logger.info(par[0]);
			}
		}
	}
}
