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

public class DVector {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected DVector(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(DVector obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_DVector(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public DVector() {
    this(yarpJNI.new_DVector__SWIG_0(), true);
  }

  public DVector(long n) {
    this(yarpJNI.new_DVector__SWIG_1(n), true);
  }

  public long size() {
    return yarpJNI.DVector_size(swigCPtr, this);
  }

  public long capacity() {
    return yarpJNI.DVector_capacity(swigCPtr, this);
  }

  public void reserve(long n) {
    yarpJNI.DVector_reserve(swigCPtr, this, n);
  }

  public boolean isEmpty() {
    return yarpJNI.DVector_isEmpty(swigCPtr, this);
  }

  public void clear() {
    yarpJNI.DVector_clear(swigCPtr, this);
  }

  public void add(double x) {
    yarpJNI.DVector_add(swigCPtr, this, x);
  }

  public double get(int i) {
    return yarpJNI.DVector_get(swigCPtr, this, i);
  }

  public void set(int i, double val) {
    yarpJNI.DVector_set(swigCPtr, this, i, val);
  }

}
