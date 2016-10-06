/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class TypedReaderCallbackImageMono {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected TypedReaderCallbackImageMono(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(TypedReaderCallbackImageMono obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_TypedReaderCallbackImageMono(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void onRead(ImageMono datum) {
    yarpJNI.TypedReaderCallbackImageMono_onRead__SWIG_0(swigCPtr, this, ImageMono.getCPtr(datum), datum);
  }

  public void onRead(ImageMono datum, TypedReaderImageMono reader) {
    yarpJNI.TypedReaderCallbackImageMono_onRead__SWIG_1(swigCPtr, this, ImageMono.getCPtr(datum), datum, TypedReaderImageMono.getCPtr(reader), reader);
  }

  public TypedReaderCallbackImageMono() {
    this(yarpJNI.new_TypedReaderCallbackImageMono(), true);
  }

}