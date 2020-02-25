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
package asap.eyepiengine.planunit;

import hmi.util.Resources;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.eyepiengine.EyePiPlanner;
import asap.eyepiengine.loader.EyePiEmbodiment;

/**
 * Takes ros message template from file, and replaces ${parameter} fragments in the file with the value of the parameters that you provide to this EPU.
 * @author Dennis
 *
 */
public class ParametrizedRosMessageEPU implements EyePiUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(ParametrizedRosMessageEPU.class.getName());

    private String wuId;
    
    private String templateFile = "";
    private String templatePath = "";
    private String topic = "";
    
    /** set by a call to prepareUnit(), based upon the various parameters */
    private String rosMsgContent = "";
    
    private Map<String,String> templateParameterMap = new HashMap<String,String>(); 

	private EyePiEmbodiment we;

    public ParametrizedRosMessageEPU()
    {
        KeyPosition start = new KeyPosition("start", 0d, 1d);
        KeyPosition end = new KeyPosition("end", 1d, 1d);
        addKeyPosition(start);
        addKeyPosition(end);
    }

    public void setEmbodiment(EyePiEmbodiment we)
    {
    	this.we = we;
    }
    
    @Override
    public void setFloatParameterValue(String name, float value) throws ParameterException
    {
    	//
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
    {
    	if (name.equals("templateFile")) templateFile = value;
    	else if (name.equals("templatePath")) templatePath = value;
    	else if (name.equals("topic")) topic = value;
    	else templateParameterMap.put(name,value);
    }

    @Override
    public String getParameterValue(String name) throws ParameterException
    {
    	if (name.equals("templateFile")) return templateFile;
    	else if (name.equals("templatePath")) return templatePath;
    	else if (name.equals("topic")) return topic;
    	else if (templateParameterMap.containsKey(name)) return templateParameterMap.get(name);
    	else return "";
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
        return 0;
    }

    @Override
    public boolean hasValidParameters()
    {
    	//TODO check if content is well formed
    	
        return !(templateFile==null||templateFile.equals("")) && !topic.equals("");
    }

    /** start the unit. */
    public void startUnit(double time) throws EPUPlayException
    {
    	we.sendRosMessage(rosMsgContent, topic);
    }

    /**
     * Ideally, we'd constantly monitor whether the current behavior has finished playing on the 
     * robot. But in this generic class we don't know what the message is, let alone being able
     * to know when the requested behavior finised
     * @param t
     *            execution time, 0 &lt t &lt 1
     * @throws EPUPlayException
     *             if the play fails for some reason
     */
    public void play(double t) throws EPUPlayException
    {
    }

    public void cleanup()
    {
    }

    @Override
    public TimedEyePiUnit createTEPU(FeedbackManager bfm, BMLBlockPeg bbPeg, String bmlId, String id)
    {
        this.wuId = id;
        templateParameterMap.put("id",id);
        return new TimedEyePiUnit(bfm, bbPeg, bmlId, id, this);
    }

    /**
     * Create a copy of this ros unit and set its parameters
     */
    @Override
    public EyePiUnit copy(EyePiEmbodiment eyepiEmbodiment)
    {
        ParametrizedRosMessageEPU result = new ParametrizedRosMessageEPU();
        result.setEmbodiment(eyepiEmbodiment);
        try 
        {
        	result.setParameterValue("topic", topic);
        	result.setParameterValue("templateFile", templateFile);
        	result.setParameterValue("templatePath", templatePath);
            result.templateParameterMap = new HashMap<String,String>();
            result.templateParameterMap.putAll(templateParameterMap);
        } catch (ParameterException ex)
        {
        	logger.error("Unexplainable error: cannot set content or topic of a ParametrizedRosMessageEPU");
        }
        for (KeyPosition keypos : getKeyPositions())
        {
            result.addKeyPosition(keypos.deepCopy());
        }
        return result;
    }

	public void prepareUnit() 
	{
		try 
		{
            Resources r = new Resources(templatePath);
            String fileContents = r.read(templateFile);
            for (Map.Entry<String, String> entry : templateParameterMap.entrySet())
            {
            	fileContents = fileContents.replaceAll("\\$\\{"+entry.getKey()+"\\}", entry.getValue());            	
            }
            rosMsgContent = fileContents;
		}
		catch (IOException ex)
		{
			logger.error("Could not fill template. Path: {} File: {}", templatePath, templateFile);
			rosMsgContent = "";
		}
	}


	//=  = = = = = = =  dealing with duration and completion

    /**
     * @return Prefered duration (in seconds) of this face unit, 0 means not determined/infinite
     */
    public double getPreferedDuration()
    {
        return 5000d;
    }


    	
    @Override
    public void addKeyPosition(KeyPosition kp)
    {
        keyPositionManager.addKeyPosition(kp);
    }

    @Override
    public KeyPosition getKeyPosition(String name)
    {
        return keyPositionManager.getKeyPosition(name);
    }

    @Override
    public List<KeyPosition> getKeyPositions()
    {
        return keyPositionManager.getKeyPositions();
    }

    @Override
    public void setKeyPositions(List<KeyPosition> p)
    {
        keyPositionManager.setKeyPositions(p);
    }

    @Override
    public void removeKeyPosition(String id)
    {
        keyPositionManager.removeKeyPosition(id);
    }

}
