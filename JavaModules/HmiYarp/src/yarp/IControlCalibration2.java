/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class IControlCalibration2 {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected IControlCalibration2(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(IControlCalibration2 obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_IControlCalibration2(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public boolean calibrate2(int axis, long type, double p1, double p2, double p3) {
    return yarpJNI.IControlCalibration2_calibrate2(swigCPtr, this, axis, type, p1, p2, p3);
  }

  public boolean done(int j) {
    return yarpJNI.IControlCalibration2_done(swigCPtr, this, j);
  }

  public boolean setCalibrator(SWIGTYPE_p_ICalibrator c) {
    return yarpJNI.IControlCalibration2_setCalibrator(swigCPtr, this, SWIGTYPE_p_ICalibrator.getCPtr(c));
  }

  public boolean calibrate() {
    return yarpJNI.IControlCalibration2_calibrate(swigCPtr, this);
  }

  public boolean park(boolean wait) {
    return yarpJNI.IControlCalibration2_park__SWIG_0(swigCPtr, this, wait);
  }

  public boolean park() {
    return yarpJNI.IControlCalibration2_park__SWIG_1(swigCPtr, this);
  }

  public boolean abortCalibration() {
    return yarpJNI.IControlCalibration2_abortCalibration(swigCPtr, this);
  }

  public boolean abortPark() {
    return yarpJNI.IControlCalibration2_abortPark(swigCPtr, this);
  }

}