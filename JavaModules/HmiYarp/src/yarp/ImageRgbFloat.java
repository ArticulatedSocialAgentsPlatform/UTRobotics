/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class ImageRgbFloat extends Image {
  private long swigCPtr;

  protected ImageRgbFloat(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.ImageRgbFloat_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ImageRgbFloat obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_ImageRgbFloat(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public int getPixelSize() {
    return yarpJNI.ImageRgbFloat_getPixelSize(swigCPtr, this);
  }

  public int getPixelCode() {
    return yarpJNI.ImageRgbFloat_getPixelCode(swigCPtr, this);
  }

  public PixelRgbFloat pixel(int x, int y) {
    return new PixelRgbFloat(yarpJNI.ImageRgbFloat_pixel(swigCPtr, this, x, y), false);
  }

  public PixelRgbFloat access(int x, int y) {
    return new PixelRgbFloat(yarpJNI.ImageRgbFloat_access(swigCPtr, this, x, y), false);
  }

  public PixelRgbFloat safePixel(int x, int y) {
    return new PixelRgbFloat(yarpJNI.ImageRgbFloat_safePixel__SWIG_0(swigCPtr, this, x, y), false);
  }

  public ImageRgbFloat() {
    this(yarpJNI.new_ImageRgbFloat(), true);
  }

}
