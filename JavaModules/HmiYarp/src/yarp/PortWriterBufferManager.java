/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PortWriterBufferManager {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PortWriterBufferManager(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PortWriterBufferManager obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PortWriterBufferManager(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void onCompletion(SWIGTYPE_p_void tracker) {
    yarpJNI.PortWriterBufferManager_onCompletion(swigCPtr, this, SWIGTYPE_p_void.getCPtr(tracker));
  }

}
