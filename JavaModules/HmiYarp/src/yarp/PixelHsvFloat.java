/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PixelHsvFloat {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PixelHsvFloat(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PixelHsvFloat obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PixelHsvFloat(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setH(float value) {
    yarpJNI.PixelHsvFloat_h_set(swigCPtr, this, value);
  }

  public float getH() {
    return yarpJNI.PixelHsvFloat_h_get(swigCPtr, this);
  }

  public void setS(float value) {
    yarpJNI.PixelHsvFloat_s_set(swigCPtr, this, value);
  }

  public float getS() {
    return yarpJNI.PixelHsvFloat_s_get(swigCPtr, this);
  }

  public void setV(float value) {
    yarpJNI.PixelHsvFloat_v_set(swigCPtr, this, value);
  }

  public float getV() {
    return yarpJNI.PixelHsvFloat_v_get(swigCPtr, this);
  }

  public PixelHsvFloat() {
    this(yarpJNI.new_PixelHsvFloat(), true);
  }

}
