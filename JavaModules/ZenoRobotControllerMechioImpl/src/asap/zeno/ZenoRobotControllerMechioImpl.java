package asap.zeno;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Properties;

import javax.annotation.concurrent.GuardedBy;

import org.jflux.api.common.rk.config.VersionProperty;
import org.jflux.api.common.rk.position.NormalizedDouble;
import org.jflux.api.core.Listener;
import org.mechio.api.animation.Animation;
import org.mechio.api.animation.Channel;
import org.mechio.api.animation.MotionPath;
import org.mechio.api.animation.messaging.RemoteAnimationPlayerClient;
import org.mechio.api.animation.player.AnimationJob;
import org.mechio.api.animation.player.AnimationJobListener;
import org.mechio.api.animation.protocol.AnimationSignal;
import org.mechio.api.motion.Joint;
import org.mechio.api.motion.Robot;
import org.mechio.api.motion.Robot.JointId;
import org.mechio.api.motion.Robot.RobotPositionHashMap;
import org.mechio.api.motion.Robot.RobotPositionMap;
import org.mechio.api.motion.messaging.RemoteJoint;
import org.mechio.api.motion.messaging.RemoteRobot;
import org.mechio.api.speech.SpeechEvent;
import org.mechio.api.speech.SpeechEventList;
import org.mechio.api.speech.messaging.RemoteSpeechServiceClient;
import org.mechio.api.speech.utils.DefaultSpeechJob;
import org.mechio.client.basic.MechIO;
import org.mechio.client.basic.R25RobotJoints;
import org.mechio.client.basic.UserSettings;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.zeno.api.ZenoRobotController;
import asap.zeno.api.ZenoSpeechListener;
import hmi.util.Resources;
import lombok.extern.slf4j.Slf4j;

/**
 * Mechio implementation of ZenoRobotController interface, controlling the Zeno R25 Robot. 
 *
 * @author Dennis Reidsma
 * @author Daniel Davison
 * @author Samuel Fernando
 */
public class ZenoRobotControllerMechioImpl implements ZenoRobotController
{
	private static Logger logger = LoggerFactory.getLogger(ZenoRobotControllerMechioImpl.class.getName());

	private static double TOO_CLOSE_THRESHOLD = 0.1;
	
	int tmpCounter = 0;
	RemoteRobot myRobot;
    
	RemoteSpeechServiceClient mySpeaker;

    RemoteAnimationPlayerClient animPlayer;

    AnimationJob currentAnimationJob;
    
    RobotPositionMap defaultPositions;
    HashMap<String, JointId> jointFromName;
    private final double EPSILON = 0.002d;
    Properties tdcJointMap;
    
    DefaultSpeechJob currentSpeechJob;

    HashMap<String,String> mechiospeechidToSpeakid = new HashMap<>();

    @GuardedBy("zenoSpeechListeners")
    private final List<ZenoSpeechListener> zenoSpeechListeners = Collections.synchronizedList(new ArrayList<ZenoSpeechListener>());

	private String animationLocation;
    
    /** zeno seems to work on port 5672 */
    public ZenoRobotControllerMechioImpl(String robotIP, String robotID, String animationLocation)
    {
    	this.animationLocation = animationLocation;
    	
        UserSettings.setSpeechAddress(robotIP);
        UserSettings.setRobotId(robotID);
        UserSettings.setRobotAddress(robotIP);
        UserSettings.setAnimationAddress(robotIP);
        myRobot = MechIO.connectRobot();
        
        mySpeaker = MechIO.connectSpeechService();
        mySpeaker.addSpeechEventListener(new SpeechJobListener());
        
        animPlayer = MechIO.connectAnimationPlayer();
        
        animPlayer.addAnimationSignalListener(new AnimPlayerListener());
        
        myRobot.addPropertyChangeListener(new PropChangeListener());
        logger.debug("Registering Animation Signal Listener");
        List<RemoteJoint> jlist = myRobot.getJointList();
        jointFromName = new HashMap<String, JointId>();
        for (RemoteJoint joint : jlist) {
            String name = joint.getName();
            String id_string = joint.getId().toString();
            logger.debug("Adding joint for {} \t {}", name, id_string);
            Joint.Id id = joint.getId();
            Robot.JointId myjid = new Robot.JointId(myRobot.getRobotId(), id);
            jointFromName.put(name, myjid);
        }

        defaultPositions = myRobot.getDefaultPositions();
        
        tdcJointMap = new Properties();
		//current jointmapfile is incomplete -- ask SAM for his file
        InputStream input = null;
		try {
	 
			input = new Resources("").getInputStream("tdc.jointmap");
			if (input == null) {
				logger.error("Sorry, unable to find properties file: tdc.jointmap");
			} else {
				//load the actual properties
				tdcJointMap.load(input);
			}

		} catch (IOException ex) {
			ex.printStackTrace();
		} finally {
			if (input != null) {
				try {
					input.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
		//TODO: read the collection of default animations (loaded in animationsByName) from a properties file containing short name and file name 

    }
    
    /**
     * Returns whether we are (still) connected to the remote Mechio Robot
     * @return the status of the connection
     */
    public boolean isConnected(){
    	return myRobot.isConnected();
    }
    
	@Override
	public void speak(String id, String text)
    {
		currentSpeechJob = (DefaultSpeechJob) mySpeaker.speak(text);
		logger.debug("Speech job sent. Speech id {}. Zeno request id {}. ",id, currentSpeechJob.getSpeechRequest().getRequestSourceId());
		mechiospeechidToSpeakid.put(currentSpeechJob.getSpeechRequest().getRequestSourceId(), id);
    }

	
	class SpeechJobListener implements  Listener<SpeechEventList<SpeechEvent>>
	{
		/**
		 * receive speech events of the ongoing speech
		 */
		@Override
		public void handleEvent(SpeechEventList<SpeechEvent> input) {
	        logger.debug("Recevend speech events {}", input);
	        List<SpeechEvent> events = input.getSpeechEvents();
	        for (SpeechEvent event : events) {
	            String type = event.getEventType();
	            logger.debug("Speech event type: {}",  event.getEventType());
	            if (type.equals("SPEECH_START")) {
	            	String speakId = mechiospeechidToSpeakid.get(Long.toString(event.getStreamNumber()));
	            	if (speakId != null)
	            		sendSpeechStart(speakId);
	            	else
	            		logger.error("Getting feedback from mechio speech but the speakID was already removed. Duplicate speech?");
	            		
	            }
	            if (type.equals("SPEECH_END")) {
	            	String speakId = mechiospeechidToSpeakid.get(Long.toString(event.getStreamNumber()));	
	            	if (speakId != null)
	            		sendSpeechEnd(speakId);
	            	else
	            		logger.error("Getting feedback from mechio speech but the speakID was already removed. Duplicate speech?");
	            	mechiospeechidToSpeakid.remove(event.getStreamNumber());
	            }
	        }
	    }
	}

	class MyAnimationJobListener implements AnimationJobListener
	{

		@Override
		public void animationAdvanced(long time) {
			logger.warn("animation progressed", time);
			
		}

		@Override
		public void animationStart(long time, Long expectedEnd) {
			logger.warn("animation start {}-{}", time, expectedEnd);
			
		}
		
	}
	@Override

	public double getAnimationDurationByName(String name) {

        if (name.equals("Default")) {
            myRobot.move(defaultPositions, 1000);
            return 1.0d;
        } else {
        	System.out.println("playing animation file: "+animationLocation+name+".xml");
            Animation animation = MechIO.loadAnimation(animationLocation+name+".xml");
            currentAnimationJob = animPlayer.playAnimation(animation);
            currentAnimationJob.pause(0);
            double dur = currentAnimationJob.getRemainingTime(System.currentTimeMillis())/1000d;
            currentAnimationJob.stop(0);
            return dur;
        }
    }


	/** in practice, this function looks up the animation in resource/animations/<name>.xml --but this may be liely to change in the future! */
	@Override
    public double playAnimationByName(String name) {

        if (name.equals("Default")) {
            myRobot.move(defaultPositions, 1000);
            return 1.0d;
        } else {
        	System.out.println("playing animation file: "+animationLocation+name+".xml");
            Animation animation = MechIO.loadAnimation(animationLocation+name+".xml");
            currentAnimationJob = animPlayer.playAnimation(animation);
            currentAnimationJob.addAnimationListener(new MyAnimationJobListener());
            logger.debug("anim duration {}", currentAnimationJob.getRemainingTime(System.currentTimeMillis()));
            return currentAnimationJob.getRemainingTime(System.currentTimeMillis())/1000d;
        }
    }

	@Override
    public double playAnimationByFileName(String fileName) {
		Animation animation = MechIO.loadAnimation(fileName);
        currentAnimationJob = animPlayer.playAnimation(animation);
        currentAnimationJob.addAnimationListener(new MyAnimationJobListener());
        logger.debug("anim duration {}", currentAnimationJob.getRemainingTime(System.currentTimeMillis()));
        return currentAnimationJob.getRemainingTime(System.currentTimeMillis())/1000d;
    }

	/** UNTESTED */
	@Override
    public double playAnimationByContent(String xmlContent) {
		try
		{
			String tempFileName = "tempanimation"+java.lang.System.currentTimeMillis()+(tmpCounter++);
			File tempDir = new File(System.getProperty("java.io.tmpdir"));
		    File tempFile = File.createTempFile(tempFileName, ".xml", tempDir);
		    FileWriter fileWriter = new FileWriter(tempFile, true);
		    BufferedWriter bw = new BufferedWriter(fileWriter);
		    bw.write(xmlContent);
		    bw.close();
	        Animation animation = MechIO.loadAnimation (tempFile.getAbsolutePath());
            currentAnimationJob = animPlayer.playAnimation(animation);
            currentAnimationJob.addAnimationListener(new MyAnimationJobListener());
            logger.debug("anim duration {}", currentAnimationJob.getRemainingTime(System.currentTimeMillis()));
            return currentAnimationJob.getRemainingTime(System.currentTimeMillis())/1000d;
		}
		catch (Exception e)
		{
			logger.error("error getting animation by content loaded: {}",e);
		}
		return 0.0d;
    }
	
	@Override
	public void stopAnimation() {
		logger.debug("Stopping currently running animation job");
		if(currentAnimationJob != null) {
			currentAnimationJob.pause(0);
			currentAnimationJob.stop(0);
		}
	}
	
    public AnimationJob moveRobot(RobotPositionMap positions, long durationMsec){
        RobotPositionMap curPositons = myRobot.getCurrentPositions();
        VersionProperty version = new VersionProperty("a"+System.currentTimeMillis(),""+System.currentTimeMillis());
        Animation anim = new Animation(version);
        for(Entry<JointId, NormalizedDouble> e : positions.entrySet()){
            Channel chan = new Channel(
                    e.getKey().getJointId().getLogicalJointNumber(), 
                    e.getKey().getJointId().toString());
            MotionPath mp = new MotionPath();
            NormalizedDouble curPos = curPositons.get(e.getKey());
            mp.addPoint(0, curPos.getValue());
            mp.addPoint(durationMsec, e.getValue().getValue());
            chan.addPath(mp);
            anim.addChannel(chan);
        }
        return animPlayer.playAnimation(anim);
    }
    
	@Override
	public void moveJointsById(Map<Integer, Double> positions, long duration)
	{
		RobotPositionMap goalPositions = new RobotPositionHashMap();
		/*
				myRobot.getGoalPositions();//new RobotPositionHashMap();
		//remove the ones that are epsilon away from their goal value
		RobotPositionMap current = myRobot.getCurrentPositions();
		ArrayList<JointId> droplist = new ArrayList<JointId>();
		for(Entry<JointId,NormalizedDouble> curpos : current.entrySet()){
			NormalizedDouble curValue = curpos.getValue();
			NormalizedDouble curGoal = goalPositions.get(curpos.getKey());
			if (Math.abs(curValue.getValue()-curGoal.getValue())<EPSILON)
			{
				droplist.add(curpos.getKey());
			}
		}
		for (JointId id : droplist)
		{
			goalPositions.remove(id);
		}
		*/
		//evaluate all specified joints, make them into normalizedDoubles
		for(Entry<Integer,Double> position : positions.entrySet()){
			JointId joint = new JointId(myRobot.getRobotId(), new Joint.Id(position.getKey()));
			NormalizedDouble value = new NormalizedDouble(position.getValue());
			goalPositions.put(joint, value);
		}
	
		//Moves the joints to the specified goal positions
		moveRobot(goalPositions, duration);
	}
	@Override
	/*
	 * 
	 * @see asap.zeno.api.ZenoRobotController#moveJointsByName(java.util.Map, long)
	 */
	public void moveJointsByName(Map<String, Double> positions, long duration)
	{
		RobotPositionMap goalPositions = new RobotPositionHashMap();
		
		/*
				myRobot.getGoalPositions();//new RobotPositionHashMap();
		//remove the ones that are epsilon away from their goal value
		RobotPositionMap current = myRobot.getCurrentPositions();
		ArrayList<JointId> droplist = new ArrayList<JointId>();
		for(Entry<JointId,NormalizedDouble> curpos : current.entrySet()){
			NormalizedDouble curValue = curpos.getValue();
			NormalizedDouble curGoal = goalPositions.get(curpos.getKey());
			if (Math.abs(curValue.getValue()-curGoal.getValue())<EPSILON)
			{
				droplist.add(curpos.getKey());
			}
		}
		for (JointId id : droplist)
		{
			goalPositions.remove(id);
		}
		*/
		//evaluate all specified joints, make them into normalizedDoubles
		for(Entry<String,Double> position : positions.entrySet()){
			JointId joint = jointFromName.get(position.getKey());
			NormalizedDouble value = new NormalizedDouble(position.getValue());
			goalPositions.put(joint, value);
		}
	 
		//Moves the joints to the specified goal positions
		moveRobot(goalPositions, duration);		
	}
	@Override
	public void moveJointsByTDCName(Map<String, Double> positions, long duration)
	{
		RobotPositionMap goalPositions = new RobotPositionHashMap();
		/*
				myRobot.getGoalPositions();//new RobotPositionHashMap();
		//remove the ones that are epsilon away from their goal value
		RobotPositionMap current = myRobot.getCurrentPositions();
		ArrayList<JointId> droplist = new ArrayList<JointId>();
		for(Entry<JointId,NormalizedDouble> curpos : current.entrySet()){
			NormalizedDouble curValue = curpos.getValue();
			NormalizedDouble curGoal = goalPositions.get(curpos.getKey());
			if (Math.abs(curValue.getValue()-curGoal.getValue())<EPSILON)
			{
				droplist.add(curpos.getKey());
			}
		}
		for (JointId id : droplist)
		{
			goalPositions.remove(id);
		}
		*/
		//evaluate all specified joints, make them into normalizedDoubles
		for(Entry<String,Double> position : positions.entrySet())
		{
			String jointName = position.getKey();
			JointId joint = jointFromName.get(tdcJointMap.getProperty(jointName));
			if (joint != null)
			{
				NormalizedDouble value = new NormalizedDouble(position.getValue());
	            //if (!jointName.equals("WAIST"))  //never take their WAIST values -- too risky
	            	goalPositions.put(joint, value);
			}
		}
	
		//Moves the joints to the specified goal positions
		moveRobot(goalPositions, duration);
		
	}
	@Override 
	public void lookAt(double x, double y)
	{
		lookAt(x,y,100);
	}
	/** simple implementation; no nice ease-in ease-out; could use some work. See also work of herwin. However, Zeno has its own easein ease out. 
//TODO: DENNIS this implementation should refuse a target that is too close to the previous target as that leads to strange motions. See below. */
	@Override 
	public void lookAt(double x, double y, long duration)
	{
		//get current joint positions for neck_yaw and neck_pitch and eye_yaw and waist
		NormalizedDouble ey = myRobot.getCurrentPositions().get(new JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.EYE_YAW)));
		NormalizedDouble newey = new NormalizedDouble(x);

		NormalizedDouble ny = myRobot.getCurrentPositions().get(new JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.NECK_YAW)));
		NormalizedDouble newny = new NormalizedDouble(x);
		
		NormalizedDouble np = myRobot.getCurrentPositions().get(new JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.NECK_PITCH)));
		NormalizedDouble newnp = new NormalizedDouble(y);
		
		NormalizedDouble w = myRobot.getCurrentPositions().get(new JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST)));
		NormalizedDouble neww = new NormalizedDouble(0.4f+(x/5));
		
        //if too close to new values, discard action
		//the above code already gives us the previous and new values. Now, we need to choose what is "too close"
		//TODO: DENNIS decide whether to do this joint by joint, or only if all of them are too close, or always do eyes and do neck/waist only if small movement?
		boolean tooClose = true;
		if ( 
				(Math.abs(newny.getValue()-ny.getValue()) > TOO_CLOSE_THRESHOLD)
				||
				(Math.abs(newnp.getValue()-np.getValue()) > TOO_CLOSE_THRESHOLD)
				||
				(Math.abs(neww.getValue()-w.getValue()) > TOO_CLOSE_THRESHOLD)
			)
		{
			tooClose = false;
			
		}
		else
		{
			logger.debug("gaze target too close to previous target: discarding");
			logger.debug("eye_yaw {} -> {}", ey, newey);
			logger.debug("neck_yaw {} -> {}", ny, newny);
			logger.debug("neck_pitch {} -> {}", np, newnp);
			logger.debug("waist {} -> {}", w, neww);
			
		}
		//not too close: execute request
        Map<Integer, Double> jointPositions = new HashMap<Integer, Double>();
		jointPositions.put(R25RobotJoints.EYE_YAW, x);
		if (!tooClose)
		{
			jointPositions.put(R25RobotJoints.NECK_YAW, x);
			jointPositions.put(R25RobotJoints.WAIST, 0.4f+(x/5));
			jointPositions.put(R25RobotJoints.NECK_PITCH, y);
		}
		
		moveJointsById(jointPositions, duration);		
	}


	@Override
	public void addSpeechListener(ZenoSpeechListener l) {
		zenoSpeechListeners.add(l);
	}

	@Override
	public void removeSpeechListener(ZenoSpeechListener l) {
		zenoSpeechListeners.remove(l);
	}

	@Override
	public void removeAllSpeechListeners() {
		zenoSpeechListeners.clear();		
	}

	private void sendSpeechStart(String speakId) {
        synchronized (zenoSpeechListeners)
        {
            for (ZenoSpeechListener l : zenoSpeechListeners)
            {
                try
                {
                    l.speechStart(speakId);
                }
                catch (Exception ex)
                {
                    logger.warn("Exception in ZenoSpeechListener: {}, start feedback", ex);
                }
            }
        }		
	}

	private void sendSpeechEnd(String speakId) {
        synchronized (zenoSpeechListeners)
        {
            for (ZenoSpeechListener l : zenoSpeechListeners)
            {
                try
                {
                    l.speechEnd(speakId);
                }
                catch (Exception ex)
                {
                    logger.warn("Exception in ZenoSpeechListener: {}, start feedback", ex);
                }
            }
        }		
	}
}

@Slf4j
class AnimPlayerListener implements Listener<AnimationSignal>
{
	public AnimPlayerListener()
	{
		log.warn("making animation listener");
		
	}

	@Override
	public void handleEvent(AnimationSignal sig) {
		log.warn("animation signal: {}", sig);
		
	}
}

@Slf4j
class PropChangeListener implements PropertyChangeListener 
{
	public PropChangeListener()
	{
		log.warn("making propchange listener");

	}

	@Override
	public void propertyChange(PropertyChangeEvent pce) {
		log.warn("pc event {}", pce);

		
	}
}
