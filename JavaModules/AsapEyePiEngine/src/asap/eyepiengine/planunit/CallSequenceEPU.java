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

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.InvalidParameterException;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.eyepiengine.EyePiPlanner;
import asap.eyepiengine.loader.EyePiEmbodiment;

/**
 * call a specific builtin RamSoc sequence to start
 * @author Dennis
 *
 */
public class CallSequenceEPU implements EyePiUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(EyePiPlanner.class.getName());

    private String wuId;
    
    private int sequenceId = 0;

    private String topic = "";

    private EyePiEmbodiment we;


    public CallSequenceEPU()
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
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
    {
    	if (name.equals("topic")) topic = value;
    	else if (name.equals("sequenceId"))
    	{
    		Integer i = null;
    		try 
    		{
    			i = Integer.valueOf(value);
    		}
    		catch (NumberFormatException e)
	        {
	            throw new InvalidParameterException(name, value);
	        }
    		
	        if (i!=null)
	        {
	            sequenceId = i.intValue();
	        }
	        else
	        {
	            throw new InvalidParameterException(name, value);
	        }
        }
        else
        {
            throw new InvalidParameterException(name, value);
        }
    }

    @Override
    public String getParameterValue(String name) throws ParameterException
    {
    	if (name.equals("topic")) return topic;
    	if (name.equals("sequenceId")) return ""+sequenceId;
    	return "" + getFloatParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
    	return 0.0f;
    }

    @Override
    public boolean hasValidParameters()
    {
    	return (sequenceId > 0) && !topic.equals(""); 
    }

    /** start the unit. */
    public void startUnit(double time) throws EPUPlayException
    {
    	we.callSequence(wuId,  sequenceId,  topic);
    }

    /**
     * Constantly monitor whether the current behavior has finished playing on the robot.
     * As soon as the behavior finishes a new keyposition is created which initiates appropriate feedback
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
    	we.callSequence(wuId,  1,  topic);
    	//TODO: Dennis call stop sequence here
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
        CallSequenceEPU result = new CallSequenceEPU();
        result.setEmbodiment(eyepiEmbodiment);
        try 
        {
        	result.setParameterValue("sequenceId", ""+sequenceId);
        	result.setParameterValue("topic", topic);
        } catch (ParameterException ex)
        {
        	logger.error("Unexplainable error: cannot set content or topic of a CallSequenceWU");
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
