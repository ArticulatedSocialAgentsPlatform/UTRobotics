/*******************************************************************************
 *******************************************************************************/
package asap.eyepiengine.bml;

import java.util.List;

import saiba.bml.core.Behaviour;
import saiba.bml.parser.SyncPoint;

import com.google.common.collect.ImmutableList;
/**
 * Abstract class for all EyePiEngine specific Behaviours.
 * 
 */
public abstract class EyePiBehaviour extends Behaviour
{
    public EyePiBehaviour(String bmlId)
    {
        super(bmlId);        
    }

    static final String EYEPINAMESPACE = "http://hmi.ewi.utwente.nl/eyepiengine";

    @Override
    public String getNamespace()
    {
        return EYEPINAMESPACE;
    }

    private static final List<String> DEFAULT_SYNCS = ImmutableList.of("start","end");
    public static List<String> getDefaultSyncPoints()
    {
        return DEFAULT_SYNCS;
    }

    @Override
    public void addDefaultSyncPoints()
    {
        for(String s:getDefaultSyncPoints())
        {
            addSyncPoint(new SyncPoint(bmlId, id, s));
        }        
    }
}
