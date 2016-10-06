/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class GazeEvent {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected GazeEvent(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(GazeEvent obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_GazeEvent(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setGazeEventParameters(GazeEventParameters value) {
    yarpJNI.GazeEvent_gazeEventParameters_set(swigCPtr, this, GazeEventParameters.getCPtr(value), value);
  }

  public GazeEventParameters getGazeEventParameters() {
    long cPtr = yarpJNI.GazeEvent_gazeEventParameters_get(swigCPtr, this);
    return (cPtr == 0) ? null : new GazeEventParameters(cPtr, false);
  }

  public void setGazeEventVariables(GazeEventVariables value) {
    yarpJNI.GazeEvent_gazeEventVariables_set(swigCPtr, this, GazeEventVariables.getCPtr(value), value);
  }

  public GazeEventVariables getGazeEventVariables() {
    long cPtr = yarpJNI.GazeEvent_gazeEventVariables_get(swigCPtr, this);
    return (cPtr == 0) ? null : new GazeEventVariables(cPtr, false);
  }

  public void gazeEventCallback() {
    yarpJNI.GazeEvent_gazeEventCallback(swigCPtr, this);
  }

}
