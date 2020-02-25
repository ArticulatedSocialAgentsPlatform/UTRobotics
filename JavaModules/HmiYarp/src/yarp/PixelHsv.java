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

public class PixelHsv {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PixelHsv(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PixelHsv obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PixelHsv(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setH(short value) {
    yarpJNI.PixelHsv_h_set(swigCPtr, this, value);
  }

  public short getH() {
    return yarpJNI.PixelHsv_h_get(swigCPtr, this);
  }

  public void setS(short value) {
    yarpJNI.PixelHsv_s_set(swigCPtr, this, value);
  }

  public short getS() {
    return yarpJNI.PixelHsv_s_get(swigCPtr, this);
  }

  public void setV(short value) {
    yarpJNI.PixelHsv_v_set(swigCPtr, this, value);
  }

  public short getV() {
    return yarpJNI.PixelHsv_v_get(swigCPtr, this);
  }

  public PixelHsv() {
    this(yarpJNI.new_PixelHsv(), true);
  }

}
