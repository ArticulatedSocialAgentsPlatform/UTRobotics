/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PortReport {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PortReport(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PortReport obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PortReport(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void report(SWIGTYPE_p_yarp__os__PortInfo info) {
    yarpJNI.PortReport_report(swigCPtr, this, SWIGTYPE_p_yarp__os__PortInfo.getCPtr(info));
  }

}
