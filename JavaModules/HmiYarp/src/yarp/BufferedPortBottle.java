/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class BufferedPortBottle extends Contactable {
  private long swigCPtr;

  protected BufferedPortBottle(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.BufferedPortBottle_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(BufferedPortBottle obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_BufferedPortBottle(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public BufferedPortBottle() {
    this(yarpJNI.new_BufferedPortBottle__SWIG_0(), true);
  }

  public BufferedPortBottle(Port port) {
    this(yarpJNI.new_BufferedPortBottle__SWIG_1(Port.getCPtr(port), port), true);
  }

  public boolean addOutput(String name) {
    return yarpJNI.BufferedPortBottle_addOutput__SWIG_0(swigCPtr, this, name);
  }

  public boolean addOutput(String name, String carrier) {
    return yarpJNI.BufferedPortBottle_addOutput__SWIG_1(swigCPtr, this, name, carrier);
  }

  public boolean addOutput(Contact contact) {
    return yarpJNI.BufferedPortBottle_addOutput__SWIG_2(swigCPtr, this, Contact.getCPtr(contact), contact);
  }

  public void close() {
    yarpJNI.BufferedPortBottle_close(swigCPtr, this);
  }

  public void interrupt() {
    yarpJNI.BufferedPortBottle_interrupt(swigCPtr, this);
  }

  public void resume() {
    yarpJNI.BufferedPortBottle_resume(swigCPtr, this);
  }

  public int getPendingReads() {
    return yarpJNI.BufferedPortBottle_getPendingReads(swigCPtr, this);
  }

  public Contact where() {
    return new Contact(yarpJNI.BufferedPortBottle_where(swigCPtr, this), true);
  }

  public String getName() {
    return yarpJNI.BufferedPortBottle_getName(swigCPtr, this);
  }

  public Bottle prepare() {
    return new Bottle(yarpJNI.BufferedPortBottle_prepare(swigCPtr, this), false);
  }

  public boolean unprepare() {
    return yarpJNI.BufferedPortBottle_unprepare(swigCPtr, this);
  }

  public void write(boolean forceStrict) {
    yarpJNI.BufferedPortBottle_write__SWIG_0(swigCPtr, this, forceStrict);
  }

  public void write() {
    yarpJNI.BufferedPortBottle_write__SWIG_1(swigCPtr, this);
  }

  public void writeStrict() {
    yarpJNI.BufferedPortBottle_writeStrict(swigCPtr, this);
  }

  public void waitForWrite() {
    yarpJNI.BufferedPortBottle_waitForWrite(swigCPtr, this);
  }

  public void setStrict(boolean strict) {
    yarpJNI.BufferedPortBottle_setStrict__SWIG_0(swigCPtr, this, strict);
  }

  public void setStrict() {
    yarpJNI.BufferedPortBottle_setStrict__SWIG_1(swigCPtr, this);
  }

  public Bottle read(boolean shouldWait) {
    long cPtr = yarpJNI.BufferedPortBottle_read__SWIG_0(swigCPtr, this, shouldWait);
    return (cPtr == 0) ? null : new Bottle(cPtr, false);
  }

  public Bottle read() {
    long cPtr = yarpJNI.BufferedPortBottle_read__SWIG_1(swigCPtr, this);
    return (cPtr == 0) ? null : new Bottle(cPtr, false);
  }

  public Bottle lastRead() {
    long cPtr = yarpJNI.BufferedPortBottle_lastRead(swigCPtr, this);
    return (cPtr == 0) ? null : new Bottle(cPtr, false);
  }

  public boolean isClosed() {
    return yarpJNI.BufferedPortBottle_isClosed(swigCPtr, this);
  }

  public void setReplier(PortReader reader) {
    yarpJNI.BufferedPortBottle_setReplier(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public void setReader(PortReader reader) {
    yarpJNI.BufferedPortBottle_setReader(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public void onRead(Bottle datum) {
    yarpJNI.BufferedPortBottle_onRead(swigCPtr, this, Bottle.getCPtr(datum), datum);
  }

  public void useCallback(BottleCallback callback) {
    yarpJNI.BufferedPortBottle_useCallback__SWIG_0(swigCPtr, this, BottleCallback.getCPtr(callback), callback);
  }

  public void useCallback() {
    yarpJNI.BufferedPortBottle_useCallback__SWIG_1(swigCPtr, this);
  }

  public void disableCallback() {
    yarpJNI.BufferedPortBottle_disableCallback(swigCPtr, this);
  }

  public boolean setEnvelope(PortWriter envelope) {
    return yarpJNI.BufferedPortBottle_setEnvelope(swigCPtr, this, PortWriter.getCPtr(envelope), envelope);
  }

  public boolean getEnvelope(PortReader envelope) {
    return yarpJNI.BufferedPortBottle_getEnvelope(swigCPtr, this, PortReader.getCPtr(envelope), envelope);
  }

  public int getInputCount() {
    return yarpJNI.BufferedPortBottle_getInputCount(swigCPtr, this);
  }

  public int getOutputCount() {
    return yarpJNI.BufferedPortBottle_getOutputCount(swigCPtr, this);
  }

  public boolean isWriting() {
    return yarpJNI.BufferedPortBottle_isWriting(swigCPtr, this);
  }

  public void getReport(PortReport reporter) {
    yarpJNI.BufferedPortBottle_getReport(swigCPtr, this, PortReport.getCPtr(reporter), reporter);
  }

  public void setReporter(PortReport reporter) {
    yarpJNI.BufferedPortBottle_setReporter(swigCPtr, this, PortReport.getCPtr(reporter), reporter);
  }

  public SWIGTYPE_p_void acquire() {
    long cPtr = yarpJNI.BufferedPortBottle_acquire(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_void(cPtr, false);
  }

  public void release(SWIGTYPE_p_void handle) {
    yarpJNI.BufferedPortBottle_release(swigCPtr, this, SWIGTYPE_p_void.getCPtr(handle));
  }

  public void setTargetPeriod(double period) {
    yarpJNI.BufferedPortBottle_setTargetPeriod(swigCPtr, this, period);
  }

  public SWIGTYPE_p_Type getType() {
    return new SWIGTYPE_p_Type(yarpJNI.BufferedPortBottle_getType(swigCPtr, this), true);
  }

  public void promiseType(SWIGTYPE_p_Type typ) {
    yarpJNI.BufferedPortBottle_promiseType(swigCPtr, this, SWIGTYPE_p_Type.getCPtr(typ));
  }

  public void setInputMode(boolean expectInput) {
    yarpJNI.BufferedPortBottle_setInputMode(swigCPtr, this, expectInput);
  }

  public void setOutputMode(boolean expectOutput) {
    yarpJNI.BufferedPortBottle_setOutputMode(swigCPtr, this, expectOutput);
  }

  public void setRpcMode(boolean expectRpc) {
    yarpJNI.BufferedPortBottle_setRpcMode(swigCPtr, this, expectRpc);
  }

  public Property acquireProperties(boolean readOnly) {
    long cPtr = yarpJNI.BufferedPortBottle_acquireProperties(swigCPtr, this, readOnly);
    return (cPtr == 0) ? null : new Property(cPtr, false);
  }

  public void releaseProperties(Property prop) {
    yarpJNI.BufferedPortBottle_releaseProperties(swigCPtr, this, Property.getCPtr(prop), prop);
  }

  public void includeNodeInName(boolean flag) {
    yarpJNI.BufferedPortBottle_includeNodeInName(swigCPtr, this, flag);
  }

}
