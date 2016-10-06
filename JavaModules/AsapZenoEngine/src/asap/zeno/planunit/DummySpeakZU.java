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
package asap.zeno.planunit;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.InvalidParameterException;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.zeno.ZenoPlanner;
import asap.zeno.embodiment.ZenoEmbodiment;

import com.google.common.primitives.Floats;

/**
 * Speak a sentence without speaking (used to fake lipsync. Deprecated because we now have real lipsync for Zeno
 * @author davisond
 *
 */
@Deprecated 
public class DummySpeakZU extends ZenoUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(DummySpeakZU.class.getName());

    
    // the unique id of this NU as specified in the BML
    private String zuId;

	private ZenoEmbodiment sze;
	
	private int duration = 1;
	
	/** The time between mouth movements in seconds. This basically controls how often the mouth goes up and down*/
	private static final double SPEECH_WORD_DURATION = 0.25d;
	
	/** The time it takes the mouth to go from its current position to the target position in ms*/
	private static final long MOUTH_MOVE_DURATION = 10;
	
	private double lastMouthMoveTime = 0;
	private MouthPosition lastMouthPosition = MouthPosition.CLOSED;
	
	private static final int ZENOR25_YAW_JOINT = 322;
	
	private enum MouthPosition {OPEN, CLOSED};

	private double startTime;

    public DummySpeakZU()
    {
        KeyPosition start = new KeyPosition("start", 0d, 1d);
        KeyPosition end = new KeyPosition("end", 1d, 1d);
        addKeyPosition(start);
        addKeyPosition(end);
    }

    public void setEmbodiment(ZenoEmbodiment sze)
    {
    	this.sze = sze;
    }
    
    @Override
    public void setFloatParameterValue(String name, float value) throws ParameterException
    {
    	
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
    {
    	if (name.equals("duration"))
        {
    		try{
            duration = Integer.parseInt(value);
    		}catch(NumberFormatException nfe){
                throw new InvalidParameterException(name, value);
    		}
        }
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
        if (name.equals("duration"))
        {
            return Integer.toString(duration);
        }
        return "" + getFloatParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
    		return 0;
    }

    @Override
    public boolean hasValidParameters()
    {
    	 return duration > 0;
    }

    /** start the unit. */
    public void startUnit(double time) throws ZUPlayException
    {
    	this.startTime = time;
    }

    /**
     * 
     * @param t
     *            execution time, 0 &lt t &lt 1
     * @throws ZUPlayException
     *             if the play fails for some reason
     */
    public void play(double t) throws ZUPlayException
    {
    	//current progress in seconds
    	double progress = t*duration;
    	
    	//do we need to move the mouth again now?
    	if(progress > lastMouthMoveTime+SPEECH_WORD_DURATION){
    		//should we open or close?
    		switch (lastMouthPosition){
    			case CLOSED:
    				moveMouthJoint(getRandomMouthAngle(MouthPosition.OPEN));
    				lastMouthPosition = MouthPosition.OPEN;
    			break;
    			case OPEN:
    				moveMouthJoint(getRandomMouthAngle(MouthPosition.CLOSED));
    				lastMouthPosition = MouthPosition.CLOSED;
				break;
    		}
    		
    		//update the last time we moved the mouth
        	lastMouthMoveTime = progress;

        	//is this the last mouth movement?
        	if(progress + SPEECH_WORD_DURATION > duration){
				moveMouthJoint(0.25d);
        	}
    	}
    }

    private void moveMouthJoint(double mouthPosition){
    	Map<Integer,Double> positions = new HashMap<Integer,Double>();
    	positions.put(ZENOR25_YAW_JOINT, mouthPosition);
    	sze.moveJointsById(positions, MOUTH_MOVE_DURATION);
    }
    
    /**
     * Generate a random joint angle for the specified mouth position
     * @param pos the position for which to generate the angle
     * @return
     */
    private double getRandomMouthAngle(MouthPosition pos) {
    	double r = 0;
    	switch (pos){
			case CLOSED:
				r = (Math.random() / 2);
			break;
			case OPEN:
				r = (0.5 + (Math.random() / 4));
			break;
		}
    	return r;
	}

	public void cleanup()
    {
		//always shut the mouth after movement
		moveMouthJoint(0.1d);
    }

    /**
     * Creates the TimedZenoUnit
     * 
     * @param bmlId
     *            BML block id
     * @param id
     *            behaviour id
     * 
     * @return the TPU
     */
    @Override
    public TimedZenoUnit createTZU(FeedbackManager bfm, BMLBlockPeg bbPeg, String bmlId, String id)
    {
        this.zuId = id;
        return new TimedZenoUnit(bfm, bbPeg, bmlId, id, this);
    }

    /**
     * @return Preferred duration (in seconds) of this face unit, 0 means not determined/infinite
     */
    @Override
    public double getPreferredDuration()
    {
        return duration;
    }

    /**
     * Create a copy of this zeno unit and link it to the display
     */
    @Override
    public ZenoUnit copy(ZenoEmbodiment zenoEmbodiment)
    {
        DummySpeakZU result = new DummySpeakZU();
        result.setEmbodiment(zenoEmbodiment);
       
        for (KeyPosition keypos : getKeyPositions())
        {
            result.addKeyPosition(keypos.deepCopy());
        }
        return result;
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

    public double getRigidity()
    {
    	// can not stretch; preferred duration is the only acceptable duration
    	return 1.0d;
    }

}
