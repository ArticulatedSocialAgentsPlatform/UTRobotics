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

public class TypedReaderImageRgb {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected TypedReaderImageRgb(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(TypedReaderImageRgb obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_TypedReaderImageRgb(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setStrict(boolean strict) {
    yarpJNI.TypedReaderImageRgb_setStrict__SWIG_0(swigCPtr, this, strict);
  }

  public void setStrict() {
    yarpJNI.TypedReaderImageRgb_setStrict__SWIG_1(swigCPtr, this);
  }

  public ImageRgb read(boolean shouldWait) {
    long cPtr = yarpJNI.TypedReaderImageRgb_read__SWIG_0(swigCPtr, this, shouldWait);
    return (cPtr == 0) ? null : new ImageRgb(cPtr, false);
  }

  public ImageRgb read() {
    long cPtr = yarpJNI.TypedReaderImageRgb_read__SWIG_1(swigCPtr, this);
    return (cPtr == 0) ? null : new ImageRgb(cPtr, false);
  }

  public void interrupt() {
    yarpJNI.TypedReaderImageRgb_interrupt(swigCPtr, this);
  }

  public ImageRgb lastRead() {
    long cPtr = yarpJNI.TypedReaderImageRgb_lastRead(swigCPtr, this);
    return (cPtr == 0) ? null : new ImageRgb(cPtr, false);
  }

  public boolean isClosed() {
    return yarpJNI.TypedReaderImageRgb_isClosed(swigCPtr, this);
  }

  public void useCallback(TypedReaderCallbackImageRgb callback) {
    yarpJNI.TypedReaderImageRgb_useCallback(swigCPtr, this, TypedReaderCallbackImageRgb.getCPtr(callback), callback);
  }

  public void disableCallback() {
    yarpJNI.TypedReaderImageRgb_disableCallback(swigCPtr, this);
  }

  public int getPendingReads() {
    return yarpJNI.TypedReaderImageRgb_getPendingReads(swigCPtr, this);
  }

  public String getName() {
    return yarpJNI.TypedReaderImageRgb_getName(swigCPtr, this);
  }

  public void setReplier(PortReader reader) {
    yarpJNI.TypedReaderImageRgb_setReplier(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public SWIGTYPE_p_void acquire() {
    long cPtr = yarpJNI.TypedReaderImageRgb_acquire(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_void(cPtr, false);
  }

  public void release(SWIGTYPE_p_void handle) {
    yarpJNI.TypedReaderImageRgb_release(swigCPtr, this, SWIGTYPE_p_void.getCPtr(handle));
  }

  public void setTargetPeriod(double period) {
    yarpJNI.TypedReaderImageRgb_setTargetPeriod(swigCPtr, this, period);
  }

}
