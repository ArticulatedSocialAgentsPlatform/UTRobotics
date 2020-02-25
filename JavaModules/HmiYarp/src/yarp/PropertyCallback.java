/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PropertyCallback {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PropertyCallback(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PropertyCallback obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PropertyCallback(swigCPtr);
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
    yarpJNI.PropertyCallback_change_ownership(this, swigCPtr, false);
  }

  public void swigTakeOwnership() {
    swigCMemOwn = true;
    yarpJNI.PropertyCallback_change_ownership(this, swigCPtr, true);
  }

  public void onRead(Property datum) {
    if (getClass() == PropertyCallback.class) yarpJNI.PropertyCallback_onRead__SWIG_0(swigCPtr, this, Property.getCPtr(datum), datum); else yarpJNI.PropertyCallback_onReadSwigExplicitPropertyCallback__SWIG_0(swigCPtr, this, Property.getCPtr(datum), datum);
  }

  public void onRead(Property datum, TypedReaderProperty reader) {
    if (getClass() == PropertyCallback.class) yarpJNI.PropertyCallback_onRead__SWIG_1(swigCPtr, this, Property.getCPtr(datum), datum, TypedReaderProperty.getCPtr(reader), reader); else yarpJNI.PropertyCallback_onReadSwigExplicitPropertyCallback__SWIG_1(swigCPtr, this, Property.getCPtr(datum), datum, TypedReaderProperty.getCPtr(reader), reader);
  }

  public PropertyCallback() {
    this(yarpJNI.new_PropertyCallback(), true);
    yarpJNI.PropertyCallback_director_connect(this, swigCPtr, swigCMemOwn, true);
  }

}
