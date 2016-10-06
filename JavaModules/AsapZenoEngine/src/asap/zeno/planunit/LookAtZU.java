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
import asap.realizer.pegboard.TimePeg;
import asap.realizer.planunit.InvalidParameterException;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.zeno.ZenoPlanner;
import asap.zeno.embodiment.ZenoEmbodiment;

import com.google.common.primitives.Floats;

/**
 * Sets a certain joint of the nao robot to a specified angle
 * This NU directly controls a primitive offered by the nao embodiment, which allows for a "fire-and-forget" method of changing joint rotations with a certain speed. 
 * The nao offers an additional more fine-grained control primitive with which it is possible to specify a trajectory which is then interpolated (currently not implemented)
 * @author davisond
 *
 */
public class LookAtZU extends ZenoUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private static Logger logger = LoggerFactory.getLogger(LookAtZU.class.getName());

    
    // the unique id of this NU as specified in the BML
    private String zuId;

	private ZenoEmbodiment sze;

	private float x;
	private float y;

    public LookAtZU()
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
    	if(name.equals("x")){
    		x = value;
    	} else if(name.equals("y")){
    		y = value;
    	}
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
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

    @Override
    public String getParameterValue(String name) throws ParameterException
    {
        return "" + getFloatParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
    	if(name.equals("x")){
    		return x;
    	} else if(name.equals("y")){
    		return y;
    	} else {
    		return 0;
    	}
    }

    @Override
    public boolean hasValidParameters()
    {
    	return (x >= 0.0f && x <= 1.0f) && (y >= 0.0f && y <= 1.0f);
    }

    /** start the unit. */
    public void startUnit(double time) throws ZUPlayException
    {
        if (expectedDuration == TimePeg.VALUE_UNKNOWN)
        {
            sze.lookAt(x, y,100);
            //logger.warn("setting lookat for default duration 100");
        }
        else
        {
            sze.lookAt(x, y,(int)(expectedDuration*1000)); 
            //logger.warn("setting lookat for duration {}",(int)(expectedDuration*1000));
        }
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
        return new TimedZenoUnit(bfm, bbPeg, bmlId, id, this);
    }


    /**
     * Create a copy of this zeno unit and link it to the display
     */
    @Override
    public ZenoUnit copy(ZenoEmbodiment zenoEmbodiment)
    {
        LookAtZU result = new LookAtZU();
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
     * @return Preferred duration (in seconds) of this face unit, 0 means not determined/infinite
     */
    public double getPreferredDuration()
    {
		return 0.1;
	}

    public double getRigidity()
    {
    	// can stretch if you want
    	return 0.0d;
    }
}
