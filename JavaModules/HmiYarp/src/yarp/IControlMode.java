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

public class IControlMode {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected IControlMode(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(IControlMode obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_IControlMode(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public boolean setPositionMode(int j) {
    return yarpJNI.IControlMode_setPositionMode(swigCPtr, this, j);
  }

  public boolean setVelocityMode(int j) {
    return yarpJNI.IControlMode_setVelocityMode(swigCPtr, this, j);
  }

  public boolean setTorqueMode(int j) {
    return yarpJNI.IControlMode_setTorqueMode(swigCPtr, this, j);
  }

  public boolean setImpedancePositionMode(int j) {
    return yarpJNI.IControlMode_setImpedancePositionMode(swigCPtr, this, j);
  }

  public boolean setImpedanceVelocityMode(int j) {
    return yarpJNI.IControlMode_setImpedanceVelocityMode(swigCPtr, this, j);
  }

  public boolean setOpenLoopMode(int j) {
    return yarpJNI.IControlMode_setOpenLoopMode(swigCPtr, this, j);
  }

  public boolean getControlMode(int j, SWIGTYPE_p_int mode) {
    return yarpJNI.IControlMode_getControlMode(swigCPtr, this, j, SWIGTYPE_p_int.getCPtr(mode));
  }

  public boolean getControlModes(SWIGTYPE_p_int modes) {
    return yarpJNI.IControlMode_getControlModes(swigCPtr, this, SWIGTYPE_p_int.getCPtr(modes));
  }

}
