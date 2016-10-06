/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class ImageRgba extends Image {
  private long swigCPtr;

  protected ImageRgba(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.ImageRgba_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ImageRgba obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_ImageRgba(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public int getPixelSize() {
    return yarpJNI.ImageRgba_getPixelSize(swigCPtr, this);
  }

  public int getPixelCode() {
    return yarpJNI.ImageRgba_getPixelCode(swigCPtr, this);
  }

  public PixelRgba pixel(int x, int y) {
    return new PixelRgba(yarpJNI.ImageRgba_pixel(swigCPtr, this, x, y), false);
  }

  public PixelRgba access(int x, int y) {
    return new PixelRgba(yarpJNI.ImageRgba_access(swigCPtr, this, x, y), false);
  }

  public PixelRgba safePixel(int x, int y) {
    return new PixelRgba(yarpJNI.ImageRgba_safePixel__SWIG_0(swigCPtr, this, x, y), false);
  }

  public ImageRgba() {
    this(yarpJNI.new_ImageRgba(), true);
  }

}
