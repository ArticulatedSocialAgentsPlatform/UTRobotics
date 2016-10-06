/*******************************************************************************
 * Copyright (C) 2009 Human Media Interaction, University of Twente, the Netherlands
 * 
 * This file is part of the Elckerlyc BML realizer.
 * 
 * Elckerlyc is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Elckerlyc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Elckerlyc.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
package asap.eyepiengine.planunit;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.google.common.primitives.Floats;

import asap.realizer.planunit.InvalidParameterException;
import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.eyepiengine.EyePiPlanner;
import asap.eyepiengine.loader.EyePiEmbodiment;

/**
 * shift "permanent default target" to x,y (between -1,1)
 * @author Dennis
 *
 */
public class GazeShiftEPU implements EyePiUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(EyePiPlanner.class.getName());

    private String wuId;
    
    private float x = 0.0f;
    private float y = 0.0f;

    private String topic = "";
    private float weight = 1000;

    private EyePiEmbodiment we;


    public GazeShiftEPU()
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
    	if (name.equals("x")) x=value;
    	if (name.equals("y")) y=value;
    	if (name.equals("weight")) weight=value;
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
    {
    	if (name.equals("topic")) topic = value;
    	else
    	{
	    	Float f = Floats.tryParse(value);
	        if (f!=null)
	        {
	            setFloatParameterValue(name, f);
	        }
	        else
	        {
	            throw new InvalidParameterException(name, value);
	        }
        }
    }

    @Override
    public String getParameterValue(String name) throws ParameterException
    {
    	if (name.equals("topic")) return topic;
    	return "" + getFloatParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
    	if (name.equals("x")) return x;
    	if (name.equals("y")) return y;
    	if (name.equals("weight")) return weight;
    	return 0.0f;
    }

    @Override
    public boolean hasValidParameters()
    {
    	return (x >= -1.0f && x <= 1.0f) && (y >= -1.0f && y <= 1.0f) && !topic.equals("") && weight >= 0; 
    }

    /** start the unit. */
    public void startUnit(double time) throws EPUPlayException
    {
    	we.setGazeShiftCoords(x, y, weight, wuId, topic);
    }

    /**
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
        return new TimedEyePiUnit(bfm, bbPeg, bmlId, id, this);
    }

    /**
     * Create a copy of this ros unit and set its parameters
     */
    @Override
    public EyePiUnit copy(EyePiEmbodiment eyepiEmbodiment)
    {
        GazeShiftEPU result = new GazeShiftEPU();
        result.setEmbodiment(eyepiEmbodiment);
        try 
        {
        	result.setFloatParameterValue("x", x);
        	result.setFloatParameterValue("y", y);
        	result.setFloatParameterValue("weight", weight);
        	result.setParameterValue("topic", topic);
        } catch (ParameterException ex)
        {
        	logger.error("Unexplainable error: cannot set content or topic of a GazeShiftEPU");
        }
        for (KeyPosition keypos : getKeyPositions())
        {
            result.addKeyPosition(keypos.deepCopy());
        }
        return result;
    }


	@Override
	public void prepareUnit() {
		// nothing needs be done here
	}
	
//=  = = = = = = =  dealing with duration and completion
    
    /**
     * @return Prefered duration (in seconds) of this face unit, 0 means not determined/infinite
     */
    public double getPreferedDuration()
    {
    	//todo
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
