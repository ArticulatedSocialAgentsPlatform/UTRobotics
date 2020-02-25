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
package asap.zeno.viseme;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.feedback.NullFeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.PegBoard;
import asap.realizer.planunit.ParameterException;
import asap.zeno.embodiment.ZenoEmbodiment;
import asap.zeno.planunit.PoseZU;
import asap.zeno.planunit.TimedZenoUnit;
import lombok.extern.slf4j.Slf4j;
import saiba.bml.core.Behaviour;

/**
 * The ZenoVisemeBinding maps from visemes to TimedZenoUnits. 
 * 
 * @author Dennis Reidsma
 */
@Slf4j
public class ZenoVisemeBinding
{
    VisemeToZenoPoseMapping mapping;
    
    public ZenoVisemeBinding(VisemeToZenoPoseMapping mapping)
    {
        this.mapping = mapping;
    }
    
    /**
     * Get a viseme unit for viseme viseme. If the viseme is not found, an 'empty' TimedZenoUnit is returned.
     */
    public TimedZenoUnit getVisemeUnit(FeedbackManager bfm,BMLBlockPeg bbPeg, Behaviour b, int viseme,
            ZenoEmbodiment ze, PegBoard pb)
    { 
        PoseZU visemeZU = new PoseZU();
        if (viseme == -1) viseme = 0;
        String pose = mapping.getPoseForViseme(viseme);
        
        if (pose != null)
        {
        	try
        	{
        		visemeZU.setParameterValue("pose", pose);
                //log.debug("viseme: {} pose: {}",viseme, pose);
        	}
        	catch (ParameterException ex)
        	{
        		log.warn("could not load pose {} for viseme {}",pose, viseme);
        	}
        }
        else
        {
            log.warn("no pose available for viseme {}",viseme);
        }

        TimedZenoUnit tzu = visemeZU.copy(ze).createTZU(bfm, bbPeg, b.getBmlId(), b.id);// from other examples, there was also one more parameter, pb);
        //TODO: DENNIS at this point, we can also set some syncpoints

        return tzu;
    }
    
    /**
     * Get a viseme unit that is not hooked up to the feedbackmanager
     * If the viseme is not found, an 'empty' TimedZenoUnit is returned.
     */
    public TimedZenoUnit getVisemeUnit(BMLBlockPeg bbPeg, Behaviour b, int viseme,            
            ZenoEmbodiment ze, PegBoard pb)
    {
        return getVisemeUnit(NullFeedbackManager.getInstance(), bbPeg, b, viseme, ze, pb);
    }
            
}
