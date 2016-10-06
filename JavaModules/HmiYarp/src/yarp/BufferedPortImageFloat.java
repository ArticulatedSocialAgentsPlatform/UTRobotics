/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class BufferedPortImageFloat extends Contactable {
  private long swigCPtr;

  protected BufferedPortImageFloat(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.BufferedPortImageFloat_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(BufferedPortImageFloat obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_BufferedPortImageFloat(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public BufferedPortImageFloat() {
    this(yarpJNI.new_BufferedPortImageFloat__SWIG_0(), true);
  }

  public BufferedPortImageFloat(Port port) {
    this(yarpJNI.new_BufferedPortImageFloat__SWIG_1(Port.getCPtr(port), port), true);
  }

  public boolean addOutput(String name) {
    return yarpJNI.BufferedPortImageFloat_addOutput__SWIG_0(swigCPtr, this, name);
  }

  public boolean addOutput(String name, String carrier) {
    return yarpJNI.BufferedPortImageFloat_addOutput__SWIG_1(swigCPtr, this, name, carrier);
  }

  public boolean addOutput(Contact contact) {
    return yarpJNI.BufferedPortImageFloat_addOutput__SWIG_2(swigCPtr, this, Contact.getCPtr(contact), contact);
  }

  public void close() {
    yarpJNI.BufferedPortImageFloat_close(swigCPtr, this);
  }

  public void interrupt() {
    yarpJNI.BufferedPortImageFloat_interrupt(swigCPtr, this);
  }

  public void resume() {
    yarpJNI.BufferedPortImageFloat_resume(swigCPtr, this);
  }

  public int getPendingReads() {
    return yarpJNI.BufferedPortImageFloat_getPendingReads(swigCPtr, this);
  }

  public Contact where() {
    return new Contact(yarpJNI.BufferedPortImageFloat_where(swigCPtr, this), true);
  }

  public String getName() {
    return yarpJNI.BufferedPortImageFloat_getName(swigCPtr, this);
  }

  public ImageFloat prepare() {
    return new ImageFloat(yarpJNI.BufferedPortImageFloat_prepare(swigCPtr, this), false);
  }

  public boolean unprepare() {
    return yarpJNI.BufferedPortImageFloat_unprepare(swigCPtr, this);
  }

  public void write(boolean forceStrict) {
    yarpJNI.BufferedPortImageFloat_write__SWIG_0(swigCPtr, this, forceStrict);
  }

  public void write() {
    yarpJNI.BufferedPortImageFloat_write__SWIG_1(swigCPtr, this);
  }

  public void writeStrict() {
    yarpJNI.BufferedPortImageFloat_writeStrict(swigCPtr, this);
  }

  public void waitForWrite() {
    yarpJNI.BufferedPortImageFloat_waitForWrite(swigCPtr, this);
  }

  public void setStrict(boolean strict) {
    yarpJNI.BufferedPortImageFloat_setStrict__SWIG_0(swigCPtr, this, strict);
  }

  public void setStrict() {
    yarpJNI.BufferedPortImageFloat_setStrict__SWIG_1(swigCPtr, this);
  }

  public ImageFloat read(boolean shouldWait) {
    long cPtr = yarpJNI.BufferedPortImageFloat_read__SWIG_0(swigCPtr, this, shouldWait);
    return (cPtr == 0) ? null : new ImageFloat(cPtr, false);
  }

  public ImageFloat read() {
    long cPtr = yarpJNI.BufferedPortImageFloat_read__SWIG_1(swigCPtr, this);
    return (cPtr == 0) ? null : new ImageFloat(cPtr, false);
  }

  public ImageFloat lastRead() {
    long cPtr = yarpJNI.BufferedPortImageFloat_lastRead(swigCPtr, this);
    return (cPtr == 0) ? null : new ImageFloat(cPtr, false);
  }

  public boolean isClosed() {
    return yarpJNI.BufferedPortImageFloat_isClosed(swigCPtr, this);
  }

  public void setReplier(PortReader reader) {
    yarpJNI.BufferedPortImageFloat_setReplier(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public void setReader(PortReader reader) {
    yarpJNI.BufferedPortImageFloat_setReader(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public void onRead(ImageFloat datum) {
    yarpJNI.BufferedPortImageFloat_onRead(swigCPtr, this, ImageFloat.getCPtr(datum), datum);
  }

  public void useCallback(TypedReaderCallbackImageFloat callback) {
    yarpJNI.BufferedPortImageFloat_useCallback__SWIG_0(swigCPtr, this, TypedReaderCallbackImageFloat.getCPtr(callback), callback);
  }

  public void useCallback() {
    yarpJNI.BufferedPortImageFloat_useCallback__SWIG_1(swigCPtr, this);
  }

  public void disableCallback() {
    yarpJNI.BufferedPortImageFloat_disableCallback(swigCPtr, this);
  }

  public boolean setEnvelope(PortWriter envelope) {
    return yarpJNI.BufferedPortImageFloat_setEnvelope(swigCPtr, this, PortWriter.getCPtr(envelope), envelope);
  }

  public boolean getEnvelope(PortReader envelope) {
    return yarpJNI.BufferedPortImageFloat_getEnvelope(swigCPtr, this, PortReader.getCPtr(envelope), envelope);
  }

  public int getInputCount() {
    return yarpJNI.BufferedPortImageFloat_getInputCount(swigCPtr, this);
  }

  public int getOutputCount() {
    return yarpJNI.BufferedPortImageFloat_getOutputCount(swigCPtr, this);
  }

  public boolean isWriting() {
    return yarpJNI.BufferedPortImageFloat_isWriting(swigCPtr, this);
  }

  public void getReport(PortReport reporter) {
    yarpJNI.BufferedPortImageFloat_getReport(swigCPtr, this, PortReport.getCPtr(reporter), reporter);
  }

  public void setReporter(PortReport reporter) {
    yarpJNI.BufferedPortImageFloat_setReporter(swigCPtr, this, PortReport.getCPtr(reporter), reporter);
  }

  public SWIGTYPE_p_void acquire() {
    long cPtr = yarpJNI.BufferedPortImageFloat_acquire(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_void(cPtr, false);
  }

  public void release(SWIGTYPE_p_void handle) {
    yarpJNI.BufferedPortImageFloat_release(swigCPtr, this, SWIGTYPE_p_void.getCPtr(handle));
  }

  public void setTargetPeriod(double period) {
    yarpJNI.BufferedPortImageFloat_setTargetPeriod(swigCPtr, this, period);
  }

  public SWIGTYPE_p_Type getType() {
    return new SWIGTYPE_p_Type(yarpJNI.BufferedPortImageFloat_getType(swigCPtr, this), true);
  }

  public void promiseType(SWIGTYPE_p_Type typ) {
    yarpJNI.BufferedPortImageFloat_promiseType(swigCPtr, this, SWIGTYPE_p_Type.getCPtr(typ));
  }

  public void setInputMode(boolean expectInput) {
    yarpJNI.BufferedPortImageFloat_setInputMode(swigCPtr, this, expectInput);
  }

  public void setOutputMode(boolean expectOutput) {
    yarpJNI.BufferedPortImageFloat_setOutputMode(swigCPtr, this, expectOutput);
  }

  public void setRpcMode(boolean expectRpc) {
    yarpJNI.BufferedPortImageFloat_setRpcMode(swigCPtr, this, expectRpc);
  }

  public Property acquireProperties(boolean readOnly) {
    long cPtr = yarpJNI.BufferedPortImageFloat_acquireProperties(swigCPtr, this, readOnly);
    return (cPtr == 0) ? null : new Property(cPtr, false);
  }

  public void releaseProperties(Property prop) {
    yarpJNI.BufferedPortImageFloat_releaseProperties(swigCPtr, this, Property.getCPtr(prop), prop);
  }

  public void includeNodeInName(boolean flag) {
    yarpJNI.BufferedPortImageFloat_includeNodeInName(swigCPtr, this, flag);
  }

}
