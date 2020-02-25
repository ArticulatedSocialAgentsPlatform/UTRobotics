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
package asap.eyepiengine;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import saiba.bml.BMLInfo;
import saiba.bml.core.Behaviour;
import saiba.bml.core.FaceLexemeBehaviour;
import saiba.bml.core.GazeBehaviour;
import saiba.bml.core.GestureBehaviour;
import saiba.bml.core.PostureBehaviour;
import saiba.bml.core.PostureShiftBehaviour;
import saiba.bml.core.ext.FaceFacsBehaviour;
import asap.realizer.AbstractPlanner;
import asap.realizer.BehaviourPlanningException;
import asap.realizer.SyncAndTimePeg;
import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.OffsetPeg;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.PlanManager;
import asap.realizer.scheduler.TimePegAndConstraint;
import asap.eyepiengine.bml.DirectRosMessageBehavior;
import asap.eyepiengine.bml.EyePiGazeBehaviour;
import asap.eyepiengine.bml.EyePiGazeShiftBehaviour;
import asap.eyepiengine.planunit.TimedEyePiUnit;
import asap.eyepiengine.eyepibinding.EyePiBinding;

public class EyePiPlanner extends AbstractPlanner<TimedEyePiUnit>
{
    static
    {
        BMLInfo.addBehaviourType(DirectRosMessageBehavior.xmlTag(), DirectRosMessageBehavior.class);
        BMLInfo.addBehaviourType(EyePiGazeBehaviour.xmlTag(), EyePiGazeBehaviour.class);
        BMLInfo.addBehaviourType(EyePiGazeShiftBehaviour.xmlTag(), EyePiGazeShiftBehaviour.class);
    }

    @SuppressWarnings("unused")
    private static Logger logger = LoggerFactory.getLogger(EyePiPlanner.class.getName());

    private final EyePiBinding eyepiBinding;

    public EyePiPlanner(FeedbackManager bfm, EyePiBinding wb, PlanManager<TimedEyePiUnit> planManager)
    {
        super(bfm, planManager);
        eyepiBinding = wb;
    }

    public EyePiBinding getRosBinding()
    {
        return eyepiBinding;
    }

    @Override
    public List<SyncAndTimePeg> addBehaviour(BMLBlockPeg bbPeg, Behaviour b, List<TimePegAndConstraint> sacs, TimedEyePiUnit planElement)
            throws BehaviourPlanningException
    {
        TimedEyePiUnit twu;

        if (planElement == null)
        {
		//TODO: why / when does this happen? does it not always go with a non-null planelement from resolveSynchs?
            List<TimedEyePiUnit> twus = eyepiBinding.getEyePiUnit(fbManager, bbPeg, b);
            if (twus.isEmpty())
            {
                throw new BehaviourPlanningException(b, "Behavior " + b.id
                        + " could not be constructed from the EyePi binding (no matching constraints), behavior omitted.");
            }

            // for now, just add the first
            twu = twus.get(0);
            if (!twu.getEyePiUnit().hasValidParameters())
            {
                throw new BehaviourPlanningException(b, "Behavior " + b.id
                        + " could not be constructed from the EyePi binding because the parameters are not valid, behavior omitted.");
            }
        }
        else
        {
            twu = (TimedEyePiUnit) planElement;
        }

        resolveDefaultKeyPositions(b, twu);
        linkSynchs(twu, sacs);

        planManager.addPlanUnit(twu);

        return constructSyncAndTimePegs(bbPeg,b,twu);
    }

    /**
     * Make a timedeyepiunit from the binding, then try to resolve it's timing (as far as you can already do this) based upon the TimePegAndConstraint you get from the scheduler plus any demands from the unit itself (such as expected duration or non-malleability of certain timing) 
     */
    @Override
    public TimedEyePiUnit resolveSynchs(BMLBlockPeg bbPeg, Behaviour b, List<TimePegAndConstraint> sacs) throws BehaviourPlanningException
    {
		//get proposed PlanUnit/TimedUnit from binding
        List<TimedEyePiUnit> twus = eyepiBinding.getEyePiUnit(fbManager, bbPeg, b);
        if (twus.isEmpty())
        {
            throw new BehaviourPlanningException(b, "Behavior " + b.id
                    + " could not be constructed from the EyePi binding (no matching constraints), behavior omitted.");
        }
        TimedEyePiUnit twu = twus.get(0); //always take the first match from the binding

        if (!twu.getEyePiUnit().hasValidParameters()) //TODO: figure out whether there are more "invalid parameter" responses that we might give early on (such as, unknown animation command -- which should be feedback from the ROS side!)
        {
            throw new BehaviourPlanningException(b, "Behavior " + b.id
                    + " could not be constructed from the EyePi binding because the parameters are not valid, behavior omitted.");
        }

        resolveDefaultKeyPositions(b, twu);
		//aanpassen: dit moet niet gedelegeerd worden maar meer hier lokaal opgelost. Want eyepibehaviors zijn niet zomaar linearly stretcheable
        //resolver.resolveSynchs(bbPeg, b, sacs, twu);
		//resolve door: 
		// - kijk of start ergens aan hangt; vraag duration op; zet end. duration niet bekend?zet op UNKNOWN
		// - anders: kijk of  end ergens aan hangt; vraag duration op; zet start. duration niet bekend? klaag dat de eyepi dat niet kan (zie voorbeeld bij spraak in geval van stretchen)
        return twu;
    }

    public void resolveDefaultKeyPositions(Behaviour b, TimedEyePiUnit twu)
    {
        if(b instanceof GazeBehaviour || b instanceof EyePiGazeBehaviour)
        {
        	twu.resolveGazeKeyPositions();
        }        
        else if(b instanceof PostureShiftBehaviour)
        {
        	twu.resolveStartAndEndKeyPositions();
        }
        else if(b instanceof PostureBehaviour)
        {
        	twu.resolvePostureKeyPositions();
        }
        else if(b instanceof FaceLexemeBehaviour)
        {
        	twu.resolveFaceKeyPositions();
        }
        else if(b instanceof FaceFacsBehaviour)
        {
        	twu.resolveFaceKeyPositions();
        }
        else if(b instanceof GestureBehaviour)
        {
        	twu.resolveGestureKeyPositions();
		}
		else
		{
			twu.resolveStartAndEndKeyPositions();
        }
    }

    @Override
    public double getRigidity(Behaviour beh)
    {
        return 0.5;
    }

	//connect all key positions in this unit, including start and end, to their correct timepegs.
	//I guess this can be kept as it is for EyePi
    private void linkSynchs(TimedEyePiUnit twu, List<TimePegAndConstraint> sacs)
    {
        for (TimePegAndConstraint s : sacs)
        {
            for (KeyPosition kp : twu.getEyePiUnit().getKeyPositions())
            {
                if (s.syncId.equals(kp.id))
                {
                    if (s.offset == 0)
                    {
                        twu.setTimePeg(kp, s.peg);
                    }
                    else
                    {
                        twu.setTimePeg(kp, new OffsetPeg(s.peg, -s.offset));
                    }
                }
            }
        }
    }

    /**
     * beyond the directrosmessage, no special BML behaviors here -- we only take the ones we get via routing. See picture engine for how it could also go...
     */
    @Override
    public List<Class<? extends Behaviour>> getSupportedBehaviours()
    {
        List<Class<? extends Behaviour>> list = new ArrayList<Class<? extends Behaviour>>();
        list.add(DirectRosMessageBehavior.class); //note: this only concerns the *BML* behaviors -- the engine may still support numerous other planunits to fulfill, e.e., other (core) BML behaviors
        list.add(EyePiGazeBehaviour.class); //note: this only concerns the *BML* behaviors -- the engine may still support numerous other planunits to fulfill, e.e., other (core) BML behaviors
        list.add(EyePiGazeShiftBehaviour.class); //note: this only concerns the *BML* behaviors -- the engine may still support numerous other planunits to fulfill, e.e., other (core) BML behaviors
        return list;
    }

    @Override
    public List<Class<? extends Behaviour>> getSupportedDescriptionExtensions()
    {
        List<Class<? extends Behaviour>> list = new ArrayList<Class<? extends Behaviour>>();
        return list;
    }
}
