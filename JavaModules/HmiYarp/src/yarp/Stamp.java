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

public class Stamp extends Portable {
  private long swigCPtr;

  protected Stamp(long cPtr, boolean cMemoryOwn) {
    super(yarpJNI.Stamp_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Stamp obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_Stamp(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public Stamp() {
    this(yarpJNI.new_Stamp__SWIG_0(), true);
  }

  public Stamp(int count, double time) {
    this(yarpJNI.new_Stamp__SWIG_1(count, time), true);
  }

  public int getCount() {
    return yarpJNI.Stamp_getCount(swigCPtr, this);
  }

  public double getTime() {
    return yarpJNI.Stamp_getTime(swigCPtr, this);
  }

  public boolean isValid() {
    return yarpJNI.Stamp_isValid(swigCPtr, this);
  }

  public int getMaxCount() {
    return yarpJNI.Stamp_getMaxCount(swigCPtr, this);
  }

  public void update() {
    yarpJNI.Stamp_update__SWIG_0(swigCPtr, this);
  }

  public void update(double time) {
    yarpJNI.Stamp_update__SWIG_1(swigCPtr, this, time);
  }

  public boolean read(ConnectionReader connection) {
    return yarpJNI.Stamp_read(swigCPtr, this, ConnectionReader.getCPtr(connection), connection);
  }

  public boolean write(ConnectionWriter connection) {
    return yarpJNI.Stamp_write(swigCPtr, this, ConnectionWriter.getCPtr(connection), connection);
  }

}
