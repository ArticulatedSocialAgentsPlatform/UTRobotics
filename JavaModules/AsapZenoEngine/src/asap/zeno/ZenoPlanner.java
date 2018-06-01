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
package asap.zeno;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.realizer.AbstractPlanner;
import asap.realizer.BehaviourPlanningException;
import asap.realizer.SyncAndTimePeg;
import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.OffsetPeg;
import asap.realizer.pegboard.TimePeg;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.PlanManager;
import asap.realizer.scheduler.TimePegAndConstraint;
import asap.zeno.bml.AnimationBehavior;
import asap.zeno.bml.DummySpeakBehavior;
import asap.zeno.bml.ExpressionBehavior;
import asap.zeno.bml.LookAtBehavior;
import asap.zeno.bml.SpeakBehavior;
import asap.zeno.bml.StopAnimationBehavior;
import asap.zeno.planunit.TimedZenoUnit;
import asap.zeno.zenobinding.ZenoBinding;
import hmi.tts.Bookmark;
import saiba.bml.BMLInfo;
import saiba.bml.core.Behaviour;
import saiba.bml.core.FaceLexemeBehaviour;
import saiba.bml.core.GazeBehaviour;
import saiba.bml.core.GestureBehaviour;
import saiba.bml.core.PostureBehaviour;
import saiba.bml.core.PostureShiftBehaviour;
import saiba.bml.core.ext.FaceFacsBehaviour;

public class ZenoPlanner extends AbstractPlanner<TimedZenoUnit>
{
    private static final double TIMEPEG_TOLERANCE = 0.003;

    static
    {
        BMLInfo.addBehaviourType(ExpressionBehavior.xmlTag(), ExpressionBehavior.class);
        BMLInfo.addBehaviourType(LookAtBehavior.xmlTag(), LookAtBehavior.class);
        BMLInfo.addBehaviourType(SpeakBehavior.xmlTag(), SpeakBehavior.class);
        BMLInfo.addBehaviourType(DummySpeakBehavior.xmlTag(), DummySpeakBehavior.class);
        BMLInfo.addBehaviourType(AnimationBehavior.xmlTag(), AnimationBehavior.class);
        BMLInfo.addBehaviourType(StopAnimationBehavior.xmlTag(), StopAnimationBehavior.class);
    }

    @SuppressWarnings("unused")
    private static Logger logger = LoggerFactory.getLogger(ZenoPlanner.class.getName());

    private final ZenoBinding zenoBinding;

    public ZenoPlanner(FeedbackManager bfm, ZenoBinding zb, PlanManager<TimedZenoUnit> planManager)
    {
        super(bfm, planManager);
        zenoBinding = zb;
    }

    public ZenoBinding getZenoBinding()
    {
        return zenoBinding;
    }

    @Override
    public List<SyncAndTimePeg> addBehaviour(BMLBlockPeg bbPeg, Behaviour b, List<TimePegAndConstraint> sacs, TimedZenoUnit planElement)
            throws BehaviourPlanningException
    {
        TimedZenoUnit tzu;

        if (planElement == null)
        {
            List<TimedZenoUnit> tzus = zenoBinding.getZenoUnit(fbManager, bbPeg, b);
            if (tzus.isEmpty())
            {
                throw new BehaviourPlanningException(b, "Behavior " + b.id
                        + " could not be constructed from the nao binding (no matching constraints), behavior omitted.");
            }

            // for now, just add the first
            tzu = tzus.get(0);
            if (!tzu.getZenoUnit().hasValidParameters())
            {
                throw new BehaviourPlanningException(b, "Behavior " + b.id
                        + " could not be constructed from the nao binding because the parameters are not valid, behavior omitted.");
            }
        }
        else
        {
            tzu = (TimedZenoUnit) planElement;
        }

        linkSynchs(tzu, sacs);
        
        if (tzu.getEndTime() == TimePeg.VALUE_UNKNOWN)
        {
            tzu.setTimePeg("end", new OffsetPeg(tzu.getTimePeg("start"), tzu.getPreferedDuration()));
        }

        List<SyncAndTimePeg> l = constructSyncAndTimePegs(bbPeg,b,tzu);

        planManager.addPlanUnit(tzu);
        
        return l;
    }

    @Override
    public TimedZenoUnit resolveSynchs(BMLBlockPeg bbPeg, Behaviour b, List<TimePegAndConstraint> sacs) throws BehaviourPlanningException
    {
        List<TimedZenoUnit> tzus = zenoBinding.getZenoUnit(fbManager, bbPeg, b);
        if (tzus.isEmpty())
        {
            throw new BehaviourPlanningException(b, "Behavior " + b.id
                    + " could not be constructed from the zeno binding (no matching constraints), behavior omitted.");
        }
        TimedZenoUnit tzu = tzus.get(0);

        if (!tzu.getZenoUnit().hasValidParameters())
        {
            throw new BehaviourPlanningException(b, "Behavior " + b.id
                    + " could not be constructed from the zeno binding because the parameters are not valid, behavior omitted."+b);
        }

        resolveDefaultKeyPositions(b, tzu);

        // what comes hereafter is adapted from the TTSPlanner
        double startTime = bbPeg.getValue();
        boolean startFound = false;

        // resolve start time
        for (TimePegAndConstraint sac : sacs)
        {
            if (sac.syncId.equals("start") && sac.peg.getGlobalValue() != TimePeg.VALUE_UNKNOWN)
            {
                startTime = sac.peg.getGlobalValue() - sac.offset;
                startFound = true;
            }
        }
        if (!startFound)
        {
            for (TimePegAndConstraint sac : sacs)
            {
                if (sac.syncId.equals("end") && sac.peg.getGlobalValue() != TimePeg.VALUE_UNKNOWN)
                {
                    startTime = sac.peg.getGlobalValue() - tzu.getPreferedDuration() - sac.offset;
                    break;
                }
            }
        }

        // resolve start and end
        TimePegAndConstraint sacNotStart = null;

        // find a random TimePegAndConstraint to link to the start
        for (TimePegAndConstraint sac : sacs)
        {
            if (!sac.syncId.equals("start"))
            {
                sacNotStart = sac;
                break;
            }
        }

        for (TimePegAndConstraint sac : sacs)
        {
            if (sac.syncId.equals("end"))
            {
                if (sac.peg.getGlobalValue() == TimePeg.VALUE_UNKNOWN)
                {
                    sac.peg.setGlobalValue(tzu.getPreferedDuration() + startTime + sac.offset);
                }//check on stretching only for the time units that have a preferred duration
                else if (tzu.getZenoUnit().getRigidity() >= 1.0d && tzu.getPreferedDuration()>0 && Math.abs(sac.peg.getGlobalValue() - (startTime + tzu.getPreferedDuration() + sac.offset)) > TIMEPEG_TOLERANCE)
                {
                    throw new BehaviourPlanningException(b, "Stretching zeno animations is not supported yet. Possibly this can be solved by moving the animation behavior up in the BML spec?  Behavior omitted.");
                }
            }

            if (sac.syncId.equals("start") && sac.peg.getGlobalValue() == TimePeg.VALUE_UNKNOWN)
            {
                if (sac.resolveAsStartOffset)
                {
                    OffsetPeg p = (OffsetPeg) sac.peg;
                    p.setLink(sacNotStart.peg);
                    p.setOffset(startTime - sacNotStart.peg.getGlobalValue());
                }
                else
                {
                    sac.peg.setGlobalValue(startTime + sac.offset);
                }
            }
        }
        linkSynchs(tzu, sacs);

        return tzu;
    }


    public void resolveDefaultKeyPositions(Behaviour b, TimedZenoUnit tnu)
    {
        if(b instanceof GazeBehaviour)
        {
            tnu.resolveGazeKeyPositions();
        }        
        else if(b instanceof PostureShiftBehaviour)
        {
            tnu.resolveStartAndEndKeyPositions();
        }
        else if(b instanceof PostureBehaviour)
        {
            tnu.resolvePostureKeyPositions();
        }
        else if(b instanceof FaceLexemeBehaviour)
        {
            tnu.resolveFaceKeyPositions();
        }
        else if(b instanceof FaceFacsBehaviour)
        {
            tnu.resolveFaceKeyPositions();
        }
        else if(b instanceof GestureBehaviour)
        {
		    tnu.resolveGestureKeyPositions();
		}
		else
		{
            tnu.resolveStartAndEndKeyPositions();
        }
    }

    @Override
    public double getRigidity(Behaviour beh)
    {
        return 0.5;
    }

    private void linkSynchs(TimedZenoUnit tzu, List<TimePegAndConstraint> sacs)
    {
        for (TimePegAndConstraint s : sacs)
        {
            for (KeyPosition kp : tzu.getZenoUnit().getKeyPositions())
            {
                if (s.syncId.equals(kp.id))
                {
                    if (s.offset == 0)
                    {
                        tzu.setTimePeg(kp, s.peg);
                    }
                    else
                    {
                        tzu.setTimePeg(kp, new OffsetPeg(s.peg, -s.offset));
                    }
                }
            }
        }
    }

    
    @Override
    public List<Class<? extends Behaviour>> getSupportedBehaviours()
    {
        List<Class<? extends Behaviour>> list = new ArrayList<Class<? extends Behaviour>>();
        list.add(ExpressionBehavior.class);
        list.add(LookAtBehavior.class);
        list.add(SpeakBehavior.class);
        list.add(DummySpeakBehavior.class);
        list.add(AnimationBehavior.class);
        list.add(StopAnimationBehavior.class);
        return list;
    }

    @Override
    public List<Class<? extends Behaviour>> getSupportedDescriptionExtensions()
    {
        List<Class<? extends Behaviour>> list = new ArrayList<Class<? extends Behaviour>>();
        return list;
    }
}
