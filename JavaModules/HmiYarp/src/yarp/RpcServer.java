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

public class RpcServer {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected RpcServer(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(RpcServer obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_RpcServer(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public RpcServer() {
    this(yarpJNI.new_RpcServer(), true);
  }

  public boolean write(PortWriter writer, PortWriter callback) {
    return yarpJNI.RpcServer_write__SWIG_0(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortWriter.getCPtr(callback), callback);
  }

  public boolean write(PortWriter writer) {
    return yarpJNI.RpcServer_write__SWIG_1(swigCPtr, this, PortWriter.getCPtr(writer), writer);
  }

  public boolean write(PortWriter writer, PortReader reader, PortWriter callback) {
    return yarpJNI.RpcServer_write__SWIG_2(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortReader.getCPtr(reader), reader, PortWriter.getCPtr(callback), callback);
  }

  public boolean write(PortWriter writer, PortReader reader) {
    return yarpJNI.RpcServer_write__SWIG_3(swigCPtr, this, PortWriter.getCPtr(writer), writer, PortReader.getCPtr(reader), reader);
  }

  public boolean read(PortReader reader, boolean willReply) {
    return yarpJNI.RpcServer_read__SWIG_0(swigCPtr, this, PortReader.getCPtr(reader), reader, willReply);
  }

  public boolean read(PortReader reader) {
    return yarpJNI.RpcServer_read__SWIG_1(swigCPtr, this, PortReader.getCPtr(reader), reader);
  }

  public void setInputMode(boolean expectInput) {
    yarpJNI.RpcServer_setInputMode(swigCPtr, this, expectInput);
  }

  public void setOutputMode(boolean expectOutput) {
    yarpJNI.RpcServer_setOutputMode(swigCPtr, this, expectOutput);
  }

  public void setRpcMode(boolean expectRpc) {
    yarpJNI.RpcServer_setRpcMode(swigCPtr, this, expectRpc);
  }

  public Port asPort() {
    return new Port(yarpJNI.RpcServer_asPort__SWIG_0(swigCPtr, this), false);
  }

}
