/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class PolyDriver extends DeviceDriver {
  private long swigCPtr;

  protected PolyDriver(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.PolyDriver_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PolyDriver obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PolyDriver(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public PolyDriver() {
    this(yarpJNI.new_PolyDriver__SWIG_0(), true);
  }

  public PolyDriver(String txt) {
    this(yarpJNI.new_PolyDriver__SWIG_1(txt), true);
  }

  public PolyDriver(Searchable config) {
    this(yarpJNI.new_PolyDriver__SWIG_2(Searchable.getCPtr(config), config), true);
  }

  public boolean open_str(String txt) {
    return yarpJNI.PolyDriver_open_str(swigCPtr, this, txt);
  }

  public boolean open(Searchable config) {
    return yarpJNI.PolyDriver_open(swigCPtr, this, Searchable.getCPtr(config), config);
  }

  public boolean link(PolyDriver alt) {
    return yarpJNI.PolyDriver_link(swigCPtr, this, PolyDriver.getCPtr(alt), alt);
  }

  public DeviceDriver take() {
    long cPtr = yarpJNI.PolyDriver_take(swigCPtr, this);
    return (cPtr == 0) ? null : new DeviceDriver(cPtr, false);
  }

  public boolean give(DeviceDriver dd, boolean own) {
    return yarpJNI.PolyDriver_give(swigCPtr, this, DeviceDriver.getCPtr(dd), dd, own);
  }

  public boolean close() {
    return yarpJNI.PolyDriver_close(swigCPtr, this);
  }

  public boolean isValid() {
    return yarpJNI.PolyDriver_isValid(swigCPtr, this);
  }

  public Bottle getOptions() {
    return new Bottle(yarpJNI.PolyDriver_getOptions(swigCPtr, this), true);
  }

  public String getComment(String option) {
    return yarpJNI.PolyDriver_getComment(swigCPtr, this, option);
  }

  public Value getDefaultValue(String option) {
    return new Value(yarpJNI.PolyDriver_getDefaultValue(swigCPtr, this, option), true);
  }

  public Value getValue(String option) {
    return new Value(yarpJNI.PolyDriver_getValue(swigCPtr, this, option), true);
  }

  public DeviceDriver getImplementation() {
    long cPtr = yarpJNI.PolyDriver_getImplementation(swigCPtr, this);
    return (cPtr == 0) ? null : new DeviceDriver(cPtr, false);
  }

  public IFrameGrabberImage viewFrameGrabberImage() {
    long cPtr = yarpJNI.PolyDriver_viewFrameGrabberImage(swigCPtr, this);
    return (cPtr == 0) ? null : new IFrameGrabberImage(cPtr, false);
  }

  public IPositionControl viewIPositionControl() {
    long cPtr = yarpJNI.PolyDriver_viewIPositionControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IPositionControl(cPtr, false);
  }

  public IVelocityControl viewIVelocityControl() {
    long cPtr = yarpJNI.PolyDriver_viewIVelocityControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IVelocityControl(cPtr, false);
  }

  public IEncoders viewIEncoders() {
    long cPtr = yarpJNI.PolyDriver_viewIEncoders(swigCPtr, this);
    return (cPtr == 0) ? null : new IEncoders(cPtr, false);
  }

  public IPidControl viewIPidControl() {
    long cPtr = yarpJNI.PolyDriver_viewIPidControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IPidControl(cPtr, false);
  }

  public IAmplifierControl viewIAmplifierControl() {
    long cPtr = yarpJNI.PolyDriver_viewIAmplifierControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IAmplifierControl(cPtr, false);
  }

  public IControlLimits viewIControlLimits() {
    long cPtr = yarpJNI.PolyDriver_viewIControlLimits(swigCPtr, this);
    return (cPtr == 0) ? null : new IControlLimits(cPtr, false);
  }

  public ICartesianControl viewICartesianControl() {
    long cPtr = yarpJNI.PolyDriver_viewICartesianControl(swigCPtr, this);
    return (cPtr == 0) ? null : new ICartesianControl(cPtr, false);
  }

  public IGazeControl viewIGazeControl() {
    long cPtr = yarpJNI.PolyDriver_viewIGazeControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IGazeControl(cPtr, false);
  }

  public IImpedanceControl viewIImpedanceControl() {
    long cPtr = yarpJNI.PolyDriver_viewIImpedanceControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IImpedanceControl(cPtr, false);
  }

  public ITorqueControl viewITorqueControl() {
    long cPtr = yarpJNI.PolyDriver_viewITorqueControl(swigCPtr, this);
    return (cPtr == 0) ? null : new ITorqueControl(cPtr, false);
  }

  public IControlMode viewIControlMode() {
    long cPtr = yarpJNI.PolyDriver_viewIControlMode(swigCPtr, this);
    return (cPtr == 0) ? null : new IControlMode(cPtr, false);
  }

  public IOpenLoopControl viewIOpenLoopControl() {
    long cPtr = yarpJNI.PolyDriver_viewIOpenLoopControl(swigCPtr, this);
    return (cPtr == 0) ? null : new IOpenLoopControl(cPtr, false);
  }

}
