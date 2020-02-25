/*******************************************************************************
 * Copyright (C) 2009-2020 Human Media Interaction, University of Twente, the Netherlands
 *
 * This file is part of the Articulated Social Agents Platform BML realizer (ASAPRealizer).
 *
 * ASAPRealizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ASAPRealizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ASAPRealizer.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package yarp;

public class Image extends Portable {
  private long swigCPtr;

  protected Image(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.Image_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Image obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_Image(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public Image() {
    this(yarpJNI.new_Image__SWIG_0(), true);
  }

  public Image(Image alt) {
    this(yarpJNI.new_Image__SWIG_1(Image.getCPtr(alt), alt), true);
  }

  public boolean copy(Image alt) {
    return yarpJNI.Image_copy__SWIG_0(swigCPtr, this, Image.getCPtr(alt), alt);
  }

  public boolean copy(Image alt, int w, int h) {
    return yarpJNI.Image_copy__SWIG_1(swigCPtr, this, Image.getCPtr(alt), alt, w, h);
  }

  public int width() {
    return yarpJNI.Image_width(swigCPtr, this);
  }

  public int height() {
    return yarpJNI.Image_height(swigCPtr, this);
  }

  public int getPixelSize() {
    return yarpJNI.Image_getPixelSize(swigCPtr, this);
  }

  public int getPixelCode() {
    return yarpJNI.Image_getPixelCode(swigCPtr, this);
  }

  public int getRowSize() {
    return yarpJNI.Image_getRowSize(swigCPtr, this);
  }

  public int getQuantum() {
    return yarpJNI.Image_getQuantum(swigCPtr, this);
  }

  public int getPadding() {
    return yarpJNI.Image_getPadding(swigCPtr, this);
  }

  public SWIGTYPE_p_unsigned_char getRow(int r) {
    long cPtr = yarpJNI.Image_getRow(swigCPtr, this, r);
    return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_char(cPtr, false);
  }

  public SWIGTYPE_p_unsigned_char getPixelAddress(int x, int y) {
    long cPtr = yarpJNI.Image_getPixelAddress(swigCPtr, this, x, y);
    return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_char(cPtr, false);
  }

  public boolean isPixel(int x, int y) {
    return yarpJNI.Image_isPixel(swigCPtr, this, x, y);
  }

  public void zero() {
    yarpJNI.Image_zero(swigCPtr, this);
  }

  public void resize(int imgWidth, int imgHeight) {
    yarpJNI.Image_resize__SWIG_0(swigCPtr, this, imgWidth, imgHeight);
  }

  public void resize(Image alt) {
    yarpJNI.Image_resize__SWIG_1(swigCPtr, this, Image.getCPtr(alt), alt);
  }

  public void setExternal(SWIGTYPE_p_void data, int imgWidth, int imgHeight) {
    yarpJNI.Image_setExternal(swigCPtr, this, SWIGTYPE_p_void.getCPtr(data), imgWidth, imgHeight);
  }

  public SWIGTYPE_p_unsigned_char getRawImage() {
    long cPtr = yarpJNI.Image_getRawImage(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_char(cPtr, false);
  }

  public int getRawImageSize() {
    return yarpJNI.Image_getRawImageSize(swigCPtr, this);
  }

  public SWIGTYPE_p_void getIplImage() {
    long cPtr = yarpJNI.Image_getIplImage(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_void(cPtr, false);
  }

  public void wrapIplImage(SWIGTYPE_p_void iplImage) {
    yarpJNI.Image_wrapIplImage(swigCPtr, this, SWIGTYPE_p_void.getCPtr(iplImage));
  }

  public boolean read(ConnectionReader connection) {
    return yarpJNI.Image_read(swigCPtr, this, ConnectionReader.getCPtr(connection), connection);
  }

  public boolean write(ConnectionWriter connection) {
    return yarpJNI.Image_write(swigCPtr, this, ConnectionWriter.getCPtr(connection), connection);
  }

  public void setQuantum(int imgQuantum) {
    yarpJNI.Image_setQuantum(swigCPtr, this, imgQuantum);
  }

  public boolean topIsLowIndex() {
    return yarpJNI.Image_topIsLowIndex(swigCPtr, this);
  }

  public void setTopIsLowIndex(boolean flag) {
    yarpJNI.Image_setTopIsLowIndex(swigCPtr, this, flag);
  }

  public SWIGTYPE_p_p_char getRowArray() {
    long cPtr = yarpJNI.Image_getRowArray(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_p_char(cPtr, false);
  }

}
