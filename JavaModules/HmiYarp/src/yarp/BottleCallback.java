/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class BottleCallback {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected BottleCallback(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(BottleCallback obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_BottleCallback(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  protected void swigDirectorDisconnect() {
    swigCMemOwn = false;
    delete();
  }

  public void swigReleaseOwnership() {
    swigCMemOwn = false;
    yarpJNI.BottleCallback_change_ownership(this, swigCPtr, false);
  }

  public void swigTakeOwnership() {
    swigCMemOwn = true;
    yarpJNI.BottleCallback_change_ownership(this, swigCPtr, true);
  }

  public void onRead(Bottle datum) {
    if (getClass() == BottleCallback.class) yarpJNI.BottleCallback_onRead__SWIG_0(swigCPtr, this, Bottle.getCPtr(datum), datum); else yarpJNI.BottleCallback_onReadSwigExplicitBottleCallback__SWIG_0(swigCPtr, this, Bottle.getCPtr(datum), datum);
  }

  public void onRead(Bottle datum, TypedReaderBottle reader) {
    if (getClass() == BottleCallback.class) yarpJNI.BottleCallback_onRead__SWIG_1(swigCPtr, this, Bottle.getCPtr(datum), datum, TypedReaderBottle.getCPtr(reader), reader); else yarpJNI.BottleCallback_onReadSwigExplicitBottleCallback__SWIG_1(swigCPtr, this, Bottle.getCPtr(datum), datum, TypedReaderBottle.getCPtr(reader), reader);
  }

  public BottleCallback() {
    this(yarpJNI.new_BottleCallback(), true);
    yarpJNI.BottleCallback_director_connect(this, swigCPtr, swigCMemOwn, true);
  }

}
