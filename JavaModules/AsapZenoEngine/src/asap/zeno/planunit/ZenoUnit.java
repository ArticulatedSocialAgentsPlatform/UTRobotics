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
package asap.zeno.planunit;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.TimePeg;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.ParameterException;
import asap.zeno.embodiment.ZenoEmbodiment;

/**
 * Contains a set of keys that map to 'world' time to animation time.
 * @author Daniel
 */
public abstract class ZenoUnit implements KeyPositionManager
{
    protected double expectedDuration = TimePeg.VALUE_UNKNOWN;
    /** many ZUs take care of this themselves, or ignore it, but some need to know what duration they should be playing for (most notably the poseZU) */
    public void setExpectedDuration(double duration)
    {
        expectedDuration = duration;
    }
    
    abstract public void setFloatParameterValue(String name, float value)throws ParameterException;
    abstract public void setParameterValue(String name, String value)throws ParameterException;
    abstract public String getParameterValue(String name)throws ParameterException;
    abstract public float getFloatParameterValue(String name)throws ParameterException;
    
    abstract public boolean hasValidParameters();    
    
       
    /** start the unit.*/
    abstract public void startUnit(double time) throws ZUPlayException;
        
    /**
     * Executes the zeno unit
     * @param t execution time, 0 &lt t &lt 1
     * @throws ZUPlayException if the play fails for some reason
     */
    abstract public void play(double t)throws ZUPlayException;
    
    /** Clean up the unit - i.e. remove traces of this naounit */    
    abstract public void cleanup();

    /**
     * Creates the TimedNaoUnit corresponding to this nao unit
     * @param bmlId     BML block id
     * @param id         behavior id
     * @return          the TNU
     */
    abstract public TimedZenoUnit createTZU(FeedbackManager bfm, BMLBlockPeg bbPeg,String bmlId,String id);
    
    
    /**
     * @return Preferred duration (in seconds) of this nao unit, 0 means not determined/infinite 
     */
    abstract public double getPreferredDuration();
 
    /**
     * 
     * @return value between 0 and 1; 0 means "can stretch as much as you want" (e.g. lookat); 1 means "absolutely rigid" (e.g. prerec animations)
     * (stretching compared to preferred duration parameter)
     */
    abstract public double getRigidity();
    
    /**
     * Create a copy of this nao unit
     */
    abstract public ZenoUnit copy(ZenoEmbodiment zenoEmbodiment);  
}