/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PixelRgb {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PixelRgb(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PixelRgb obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PixelRgb(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setR(short value) {
    yarpJNI.PixelRgb_r_set(swigCPtr, this, value);
  }

  public short getR() {
    return yarpJNI.PixelRgb_r_get(swigCPtr, this);
  }

  public void setG(short value) {
    yarpJNI.PixelRgb_g_set(swigCPtr, this, value);
  }

  public short getG() {
    return yarpJNI.PixelRgb_g_get(swigCPtr, this);
  }

  public void setB(short value) {
    yarpJNI.PixelRgb_b_set(swigCPtr, this, value);
  }

  public short getB() {
    return yarpJNI.PixelRgb_b_get(swigCPtr, this);
  }

  public PixelRgb() {
    this(yarpJNI.new_PixelRgb__SWIG_0(), true);
  }

  public PixelRgb(short n_r, short n_g, short n_b) {
    this(yarpJNI.new_PixelRgb__SWIG_1(n_r, n_g, n_b), true);
  }

}
