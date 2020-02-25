/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class DeviceDriver extends IConfig {
  private long swigCPtr;

  protected DeviceDriver(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.DeviceDriver_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(DeviceDriver obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_DeviceDriver(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public boolean open(Searchable config) {
    return yarpJNI.DeviceDriver_open(swigCPtr, this, Searchable.getCPtr(config), config);
  }

  public boolean close() {
    return yarpJNI.DeviceDriver_close(swigCPtr, this);
  }

  public DeviceDriver getImplementation() {
    long cPtr = yarpJNI.DeviceDriver_getImplementation(swigCPtr, this);
    return (cPtr == 0) ? null : new DeviceDriver(cPtr, false);
  }

  public DeviceDriver() {
    this(yarpJNI.new_DeviceDriver(), true);
  }

}
