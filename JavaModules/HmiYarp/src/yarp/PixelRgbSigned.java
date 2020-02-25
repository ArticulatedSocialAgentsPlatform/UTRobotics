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

public class PixelRgbSigned {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected PixelRgbSigned(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PixelRgbSigned obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_PixelRgbSigned(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setR(char value) {
    yarpJNI.PixelRgbSigned_r_set(swigCPtr, this, value);
  }

  public char getR() {
    return yarpJNI.PixelRgbSigned_r_get(swigCPtr, this);
  }

  public void setG(char value) {
    yarpJNI.PixelRgbSigned_g_set(swigCPtr, this, value);
  }

  public char getG() {
    return yarpJNI.PixelRgbSigned_g_get(swigCPtr, this);
  }

  public void setB(char value) {
    yarpJNI.PixelRgbSigned_b_set(swigCPtr, this, value);
  }

  public char getB() {
    return yarpJNI.PixelRgbSigned_b_get(swigCPtr, this);
  }

  public PixelRgbSigned() {
    this(yarpJNI.new_PixelRgbSigned(), true);
  }

}
