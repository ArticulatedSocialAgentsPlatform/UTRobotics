package asap.zeno.api;

public interface ZenoSpeechListener {
	void speechStart(String id);
	void speechEnd(String id);
}
