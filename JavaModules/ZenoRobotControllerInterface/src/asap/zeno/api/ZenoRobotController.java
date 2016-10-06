package asap.zeno.api;

import java.util.Map;

/**
 * Interface definitions for ZenoRobotController, exposing the control primitives of the Zeno R25 Robot. 
 * Main implementations are (a) the Mechio implementation that actually controls a robot and (b) the 
 * ZenoEmbodiment (which serves as a middleware adapter for the robot)
 *
 * @author dennisr
 * 
 */
public interface ZenoRobotController
{
	/** Tries to immediately speak the given text, using Zeno's internal TTS. Note, though, that the robot may actually maintain an internal queue of speech requests: because it can't do simlutaneous speech, it might handle incoming requests one by one as each one completes. The id parameter is used for sending feedback about speech events such as speechend or bookmarks. */ 
	void speak(String id, String text);
	
	/** Add listener for speech events (word boundaries, speech end, ...).  */
	void addSpeechListener(ZenoSpeechListener l);
	/** Remove listener for speech events (word boundaries, speech end, ...).  */
	void removeSpeechListener(ZenoSpeechListener l);
	/** Remove all listener for speech events (word boundaries, speech end, ...).  */
	void removeAllSpeechListeners();
	
	//void interruptCurrentSpeech(); --> towards more adaptive TTS...
	
	//void addSpeech(String text); --> Sam used this to queue speech and send it to the robot one by one. However, since the robot internally maintains this queue as well, there's no reason to have this function.
	//void flushQueue() (from Sam, used to wait until finish of speech & anim, then trigger next one. Only if the function above is needed)

	double getAnimationDurationByName(String name);
	
	/** Play animation by name. Robot will search for the animation in a special cache. Returns expected duration in millis. */
	double playAnimationByName(String name);
	//--> then also needs hasanimation(name) and getAnimationNames() for at least the preloaded animations? Although this function also allows to load any other animation that can be found on the file system.

	/** Returns expected duration in millis. Play animation by file name. Does not use the Resourcesconventions of HMI, so use path like resource/../name.xml (to be changed!)  */
	double playAnimationByFileName(String fileName);

	/** Returns expected duration in millis. Directly send the XML of the animation */
	double playAnimationByContent(String xmlContent);
	
	/**
	 * Moves a set of joints to the indicated positions over a specified duration.
	 * Double values will be normalized by robot before running the command.
	 * @param positions a map from containing the joint ids (from R25RobotJoints static variables) as keys, to requested positions as values
	 * @param duration how long it should take (in milliseconds) for the joints to reach the specified position. Note sure if that has an effect --> NEEDS TESTING
	 */
	void moveJointsById(Map<Integer, Double> positions, long duration);
	/**
	 * Moves a set of joints to the indicated positions over a specified duration.
	 * Double values will be normalized by robot before running the command.
	 * @param positions a map from containing the joint names (from R25RobotJoints static variables) as keys, to requested positions as values
	 * @param duration how long it should take (in milliseconds) for the joints to reach the specified position. Note sure if that has an effect --> NEEDS TESTING
	 *
	 * {Right Grasp=myRobot::521, Left Wrist=myRobot::420, Right Lower=myRobot::501, Left Grasp=myRobot::421,
	 *  Eye Turn=myRobot::311, Brows=myRobot::300, Smile=myRobot::320, Right Upper=myRobot::500, Right Elbow=myRobot::510, 
	 *  Mouth=myRobot::322, Eyelids=myRobot::301, Waist=myRobot::100, Left Elbow=myRobot::410, Walk Cam=myRobot::600, 
	 *  Neck Yaw=myRobot::200, Right Wrist=myRobot::520, Neck Pitch=myRobot::202, Left Upper=myRobot::400, Right Foot=myRobot::610, 
	 *  Left Foot=myRobot::601, Left Lower=myRobot::401}
	 */
	void moveJointsByName(Map<String, Double> positions, long duration);
	/**
	 * Moves a set of joints to the indicated positions over a specified duration.
	 * Double values will be normalized by robot before running the command.
	 * @param positions a map from containing the joint names (in the format provided by Semmelweis TDCs) as keys, to requested positions as values
	 * @param duration how long it should take (in milliseconds) for the joints to reach the specified position. Note sure if that has an effect --> NEEDS TESTING
	 */
	void moveJointsByTDCName(Map<String, Double> positions, long duration);

	/** Look at provided x,y, which are values between 0..1 (with 0.5 meaning "centered"), achieve target in duration millis. Note that Zeno might refuse a subsequent lookattargets that is too close to the previous one as that leads to strange motions in the hardware platform */
	void lookAt(double x, double y, long duration);
	/** lookAt(x,y, 100) */
	void lookAt(double x, double y);
	
/*

animationfinished, speechfinished (if no listener solution can be found)

setpose (by name and by xml spec of full pose, intensity, duration (which will not work)) --> this will cover SEs "expressions"
interpolatePose (by xml spec of pose, intensity, duration) --> which will generate a animation xml spec (see below) and its interpolation points and will load that. will interpolate towards desired pose (and stay there -- if you want to return, sent intorpolatepose(getdefaultpositions)

- Daniels "expressions" can be done by generating an XML spec of the corresponding animation on the fly -- including all necessary interpolation values along the way)

getdefaultpositions  --> why expose this?


*/
}
