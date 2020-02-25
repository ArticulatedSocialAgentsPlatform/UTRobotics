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

public class ITorqueControl {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected ITorqueControl(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ITorqueControl obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_ITorqueControl(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public boolean getAxes(SWIGTYPE_p_int ax) {
    return yarpJNI.ITorqueControl_getAxes(swigCPtr, this, SWIGTYPE_p_int.getCPtr(ax));
  }

  public boolean setTorqueMode() {
    return yarpJNI.ITorqueControl_setTorqueMode(swigCPtr, this);
  }

  public boolean getRefTorques(SWIGTYPE_p_double t) {
    return yarpJNI.ITorqueControl_getRefTorques(swigCPtr, this, SWIGTYPE_p_double.getCPtr(t));
  }

  public boolean getRefTorque(int j, SWIGTYPE_p_double t) {
    return yarpJNI.ITorqueControl_getRefTorque(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(t));
  }

  public boolean setRefTorques(SWIGTYPE_p_double t) {
    return yarpJNI.ITorqueControl_setRefTorques(swigCPtr, this, SWIGTYPE_p_double.getCPtr(t));
  }

  public boolean setRefTorque(int j, double t) {
    return yarpJNI.ITorqueControl_setRefTorque(swigCPtr, this, j, t);
  }

  public boolean getBemfParam(int j, SWIGTYPE_p_double bemf) {
    return yarpJNI.ITorqueControl_getBemfParam(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(bemf));
  }

  public boolean setBemfParam(int j, double bemf) {
    return yarpJNI.ITorqueControl_setBemfParam(swigCPtr, this, j, bemf);
  }

  public boolean setTorquePid(int j, Pid pid) {
    return yarpJNI.ITorqueControl_setTorquePid(swigCPtr, this, j, Pid.getCPtr(pid), pid);
  }

  public boolean getTorque(int j, SWIGTYPE_p_double t) {
    return yarpJNI.ITorqueControl_getTorque(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(t));
  }

  public boolean getTorques(SWIGTYPE_p_double t) {
    return yarpJNI.ITorqueControl_getTorques(swigCPtr, this, SWIGTYPE_p_double.getCPtr(t));
  }

  public boolean getTorqueRange(int j, SWIGTYPE_p_double min, SWIGTYPE_p_double max) {
    return yarpJNI.ITorqueControl_getTorqueRange(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(min), SWIGTYPE_p_double.getCPtr(max));
  }

  public boolean getTorqueRanges(SWIGTYPE_p_double min, SWIGTYPE_p_double max) {
    return yarpJNI.ITorqueControl_getTorqueRanges(swigCPtr, this, SWIGTYPE_p_double.getCPtr(min), SWIGTYPE_p_double.getCPtr(max));
  }

  public boolean setTorquePids(Pid pids) {
    return yarpJNI.ITorqueControl_setTorquePids(swigCPtr, this, Pid.getCPtr(pids), pids);
  }

  public boolean setTorqueErrorLimit(int j, double limit) {
    return yarpJNI.ITorqueControl_setTorqueErrorLimit(swigCPtr, this, j, limit);
  }

  public boolean setTorqueErrorLimits(SWIGTYPE_p_double limits) {
    return yarpJNI.ITorqueControl_setTorqueErrorLimits(swigCPtr, this, SWIGTYPE_p_double.getCPtr(limits));
  }

  public boolean getTorqueError(int j, SWIGTYPE_p_double err) {
    return yarpJNI.ITorqueControl_getTorqueError(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(err));
  }

  public boolean getTorqueErrors(SWIGTYPE_p_double errs) {
    return yarpJNI.ITorqueControl_getTorqueErrors(swigCPtr, this, SWIGTYPE_p_double.getCPtr(errs));
  }

  public boolean getTorquePidOutput(int j, SWIGTYPE_p_double out) {
    return yarpJNI.ITorqueControl_getTorquePidOutput(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(out));
  }

  public boolean getTorquePidOutputs(SWIGTYPE_p_double outs) {
    return yarpJNI.ITorqueControl_getTorquePidOutputs(swigCPtr, this, SWIGTYPE_p_double.getCPtr(outs));
  }

  public boolean getTorquePid(int j, Pid pid) {
    return yarpJNI.ITorqueControl_getTorquePid(swigCPtr, this, j, Pid.getCPtr(pid), pid);
  }

  public boolean getTorquePids(Pid pids) {
    return yarpJNI.ITorqueControl_getTorquePids(swigCPtr, this, Pid.getCPtr(pids), pids);
  }

  public boolean getTorqueErrorLimit(int j, SWIGTYPE_p_double limit) {
    return yarpJNI.ITorqueControl_getTorqueErrorLimit(swigCPtr, this, j, SWIGTYPE_p_double.getCPtr(limit));
  }

  public boolean getTorqueErrorLimits(SWIGTYPE_p_double limits) {
    return yarpJNI.ITorqueControl_getTorqueErrorLimits(swigCPtr, this, SWIGTYPE_p_double.getCPtr(limits));
  }

  public boolean resetTorquePid(int j) {
    return yarpJNI.ITorqueControl_resetTorquePid(swigCPtr, this, j);
  }

  public boolean disableTorquePid(int j) {
    return yarpJNI.ITorqueControl_disableTorquePid(swigCPtr, this, j);
  }

  public boolean enableTorquePid(int j) {
    return yarpJNI.ITorqueControl_enableTorquePid(swigCPtr, this, j);
  }

  public boolean setTorqueOffset(int j, double v) {
    return yarpJNI.ITorqueControl_setTorqueOffset(swigCPtr, this, j, v);
  }

}
