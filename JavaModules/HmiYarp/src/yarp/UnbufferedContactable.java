/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class UnbufferedContactable extends Contactable {
  private long swigCPtr;

  protected UnbufferedContactable(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.UnbufferedContactable_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(UnbufferedContactable obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_UnbufferedContactable(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public boolean write(PortWriter writer, PortWriter callback) {
    return yarpJNI.UnbufferedContactable_write__SWIG_0(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortWriter.getCPtr(callback), callback);
  }

  public boolean write(PortWriter writer) {
    return yarpJNI.UnbufferedContactable_write__SWIG_1(swigCPtr, this, PortWriter.getCPtr(writer), writer);
  }

  public boolean write(PortWriter writer, PortReader reader, PortWriter callback) {
    return yarpJNI.UnbufferedContactable_write__SWIG_2(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortReader.getCPtr(reader), reader, PortWriter.getCPtr(callback), callback);
  }

  public boolean write(PortWriter writer, PortReader reader) {
    return yarpJNI.UnbufferedContactable_write__SWIG_3(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortReader.getCPtr(reader), reader);
  }

  public boolean read(PortReader reader, boolean willReply) {
    return yarpJNI.UnbufferedContactable_read__SWIG_0(swigCPtr, this, PortReader.getCPtr(reader), reader, willReply);
  }

  public boolean read(PortReader reader) {
    return yarpJNI.UnbufferedContactable_read__SWIG_1(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public boolean reply(PortWriter writer) {
    return yarpJNI.UnbufferedContactable_reply(swigCPtr, this, PortWriter.getCPtr(writer), writer);
  }

  public boolean replyAndDrop(PortWriter writer) {
    return yarpJNI.UnbufferedContactable_replyAndDrop(swigCPtr, this, PortWriter.getCPtr(writer), writer);
  }

}