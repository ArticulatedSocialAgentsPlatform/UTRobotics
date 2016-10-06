/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class ICalibrator {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected ICalibrator(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ICalibrator obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_ICalibrator(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public boolean calibrate(DeviceDriver dd) {
    return yarpJNI.ICalibrator_calibrate(swigCPtr, this, DeviceDriver.getCPtr(dd), dd);
  }

  public boolean park(DeviceDriver dd, boolean wait) {
    return yarpJNI.ICalibrator_park__SWIG_0(swigCPtr, this, DeviceDriver.getCPtr(dd), dd, wait);
  }

  public boolean park(DeviceDriver dd) {
    return yarpJNI.ICalibrator_park__SWIG_1(swigCPtr, this, DeviceDriver.getCPtr(dd), dd);
  }

  public boolean quitCalibrate() {
    return yarpJNI.ICalibrator_quitCalibrate(swigCPtr, this);
  }

  public boolean quitPark() {
    return yarpJNI.ICalibrator_quitPark(swigCPtr, this);
  }

}
