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

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.ParameterException;
import asap.eyepiengine.loader.EyePiEmbodiment;

/**
 * 
 * @author Dennis
 */
public interface EyePiUnit extends KeyPositionManager
{
    void setFloatParameterValue(String name, float value)throws ParameterException;
    void setParameterValue(String name, String value)throws ParameterException;
    String getParameterValue(String name)throws ParameterException;
    float getFloatParameterValue(String name)throws ParameterException;
    
    boolean hasValidParameters();    
    
    /** prepare unit, assuming that all parameters have correctly been set. may involve, e.g, caching ROS messages and other things */ 
    void prepareUnit();
    
    /** start the unit.*/
    void startUnit(double time) throws EPUPlayException;
        
    /**
     * Executes the unit (generally by sending a message somewhere)
     * @param t execution time, 0 &lt t &lt 1
     * @throws EPUPlayException if the play fails for some reason
     */
    void play(double t)throws EPUPlayException;
    
    /** Clean up the unit - i.e. remove traces of this unit */    
    void cleanup();

    /**
     * Creates the TimedUnit corresponding to this planunit
     * @param bmlId     BML block id
     * @param id         behavior id
     * @return          the TRU
     */
    TimedEyePiUnit createTEPU(FeedbackManager bfm, BMLBlockPeg bbPeg,String bmlId,String id);
        
    /**
     * @return Preferred duration (in seconds) of this unit, 0 means not determined/infinite 
     */
    //double getPreferedDuration();
    
    /**
     * Create a copy of this planunit
     */
    EyePiUnit copy(EyePiEmbodiment eyepiEmbodiment);  
}