/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class SVector {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected SVector(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(SVector obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_SVector(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public SVector() {
    this(yarpJNI.new_SVector__SWIG_0(), true);
  }

  public SVector(long n) {
    this(yarpJNI.new_SVector__SWIG_1(n), true);
  }

  public long size() {
    return yarpJNI.SVector_size(swigCPtr, this);
  }

  public long capacity() {
    return yarpJNI.SVector_capacity(swigCPtr, this);
  }

  public void reserve(long n) {
    yarpJNI.SVector_reserve(swigCPtr, this, n);
  }

  public boolean isEmpty() {
    return yarpJNI.SVector_isEmpty(swigCPtr, this);
  }

  public void clear() {
    yarpJNI.SVector_clear(swigCPtr, this);
  }

  public void add(String x) {
    yarpJNI.SVector_add(swigCPtr, this, x);
  }

  public String get(int i) {
    return yarpJNI.SVector_get(swigCPtr, this, i);
  }

  public void set(int i, String val) {
    yarpJNI.SVector_set(swigCPtr, this, i, val);
  }

}