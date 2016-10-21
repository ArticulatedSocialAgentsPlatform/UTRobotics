/*******************************************************************************
 *******************************************************************************/
package asap.zeno.lipsync;

import java.util.ArrayList;
import java.util.HashMap;

import asap.realizer.lipsync.LipSynchProvider;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.OffsetPeg;
import asap.realizer.pegboard.PegBoard;
import asap.realizer.pegboard.TimePeg;
import asap.realizer.planunit.PlanManager;
import asap.realizer.planunit.TimedPlanUnit;
import asap.zeno.embodiment.ZenoEmbodiment;
import asap.zeno.planunit.TimedZenoUnit;
import asap.zeno.viseme.ZenoVisemeBinding;
import hmi.tts.TTSTiming;
import hmi.tts.Visime;
import lombok.extern.slf4j.Slf4j;
import saiba.bml.core.Behaviour;

/**
 * Creates TimedZenonits for lipsync 
 * @author Dennis Reidsma
 *
 */
@Slf4j
public class TimedZenoUnitLipSynchProvider implements LipSynchProvider
{
    private final ZenoVisemeBinding visemeBinding;
    private final ZenoEmbodiment ze;
    private final PlanManager<TimedZenoUnit> zenoPlanManager;
    private final PegBoard pegBoard;
    
    //TODO: make this configurable
    //this value controls how much a viseme is offset when sending.. set a negative value if viseme should be sent before its being spoken
    private static final double VISEME_OFFSET = -0.2d;
    
    public TimedZenoUnitLipSynchProvider(ZenoVisemeBinding visBinding, ZenoEmbodiment ze, PlanManager<TimedZenoUnit>zenoPlanManager, PegBoard pb)
    {
        visemeBinding = visBinding;
        this.ze = ze;
        pegBoard = pb;
        this.zenoPlanManager= zenoPlanManager; 
    }
    
    @Override
    public void addLipSyncMovement(BMLBlockPeg bbPeg, Behaviour beh, TimedPlanUnit bs, TTSTiming timing)
    {
        ArrayList<TimedZenoUnit> tzus = new ArrayList<TimedZenoUnit>();
        double totalDuration = 0d;
        double prevDuration = 0d;

        // add null viseme before
        TimedZenoUnit tzu = null;
        HashMap<TimedZenoUnit, Double> startTimes = new HashMap<TimedZenoUnit, Double>();
        HashMap<TimedZenoUnit, Double> endTimes = new HashMap<TimedZenoUnit, Double>();
        
        for (Visime vis : timing.getVisimes())
        {
            // visemes lopen nu vanaf de peak van de vorige viseme tot aan de peak van deze viseme 
            double start = totalDuration / 1000d - prevDuration / 2000;
            double peak = totalDuration / 1000d + vis.getDuration() / 2000d;
            double end = totalDuration / 1000d + vis.getDuration() / 1000d;

            tzu = visemeBinding.getVisemeUnit(bbPeg, beh, vis.getNumber(), ze, pegBoard);

            startTimes.put(tzu, start);
            endTimes.put(tzu, end);
            tzus.add(tzu);
            totalDuration += vis.getDuration();
            prevDuration = vis.getDuration();
        }
        

        for (TimedZenoUnit vzu : tzus)
        {
            vzu.setSubUnit(true);
            zenoPlanManager.addPlanUnit(vzu);
        }

        // and now link viseme units to the speech timepeg!
        for (TimedZenoUnit plannedZU : tzus)
        {
            //TODO: if we're really only going to do open and closed mouths, we might conflate the TZUs that have the same pose... but that's kind of tricky to find out at this point
            TimePeg startPeg = new OffsetPeg(bs.getTimePeg("start"), startTimes.get(plannedZU) + VISEME_OFFSET);

            plannedZU.setTimePeg("start", startPeg);
            TimePeg endPeg = new OffsetPeg(bs.getTimePeg("start"), endTimes.get(plannedZU) + VISEME_OFFSET);
            plannedZU.setTimePeg("end", endPeg);
            log.debug("adding zeno pose movement at {}-{}", plannedZU.getStartTime(), plannedZU.getEndTime());
        }        
    }

    
}
