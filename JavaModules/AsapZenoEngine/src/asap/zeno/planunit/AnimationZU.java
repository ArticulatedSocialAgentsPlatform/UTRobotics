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
import asap.zeno.ZenoPlanner;
import asap.zeno.embodiment.ZenoEmbodiment;

import com.google.common.primitives.Floats;

/**
 * Play an animation from file
 * @author Dennis Reidsma
 *
 */
public class AnimationZU extends ZenoUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(AnimationZU.class.getName());

    // the unique id of this NU as specified in the BML
    private String zuId;

	private ZenoEmbodiment sze;
	
	private String animation = "";


	private double cachedExpectedDuration = -1;
    public AnimationZU()
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
    	if (name.equals("animation"))
        {
            animation = value;
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
        if (name.equals("animation"))
        {
            return animation;
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
    	 return true;
    }

    /** start the unit. */
    public void startUnit(double time) throws ZUPlayException
    {
    	double dur = sze.playAnimationByName(animation);
    	logger.debug("expected duration of animation: {}. Check whether this equals the planned duration! ",dur);
    	
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

    }

	public void cleanup()
    {
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
        TimedZenoUnit tzu = new TimedZenoUnit(bfm, bbPeg, bmlId, id, this);
        return tzu;
    }

    /**
     * Create a copy of this zeno unit and link it to the display
     */
    @Override
    public ZenoUnit copy(ZenoEmbodiment zenoEmbodiment)
    {
        AnimationZU result = new AnimationZU();
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


    /**
     * @return Prefered animation (in seconds) of this face unit, 0 means not determined/infinite
     */
    @Override
    public double getPreferredDuration()
    {
		if (cachedExpectedDuration >= 0)
		{
			return cachedExpectedDuration;
		}
        if (animation!= null && !animation.equals("")) 
        {
        	cachedExpectedDuration=sze.getAnimationDurationByName(animation);
        	logger.debug("Planning: Animation {} has duration {}",animation,cachedExpectedDuration );
        	return cachedExpectedDuration ;
        }
        else 
        {
        	logger.warn("can't give expectd duration while animation is empty or null: \"{}\"",animation);
        }
		return 0;
	}

    public double getRigidity()
    {
    	// cannot stretch at all
    	return 1.0d;
    }
}
