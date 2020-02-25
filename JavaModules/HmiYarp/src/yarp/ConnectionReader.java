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

public class ConnectionReader {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected ConnectionReader(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ConnectionReader obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        yarpJNI.delete_ConnectionReader(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public boolean expectBlock(String data, long len) {
    return yarpJNI.ConnectionReader_expectBlock(swigCPtr, this, data, len);
  }

  public String expectText(int terminatingChar) {
    return yarpJNI.ConnectionReader_expectText__SWIG_0(swigCPtr, this, terminatingChar);
  }

  public String expectText() {
    return yarpJNI.ConnectionReader_expectText__SWIG_1(swigCPtr, this);
  }

  public int expectInt() {
    return yarpJNI.ConnectionReader_expectInt(swigCPtr, this);
  }

  public double expectDouble() {
    return yarpJNI.ConnectionReader_expectDouble(swigCPtr, this);
  }

  public boolean isTextMode() {
    return yarpJNI.ConnectionReader_isTextMode(swigCPtr, this);
  }

  public boolean isBareMode() {
    return yarpJNI.ConnectionReader_isBareMode(swigCPtr, this);
  }

  public boolean convertTextMode() {
    return yarpJNI.ConnectionReader_convertTextMode(swigCPtr, this);
  }

  public long getSize() {
    return yarpJNI.ConnectionReader_getSize(swigCPtr, this);
  }

  public ConnectionWriter getWriter() {
    long cPtr = yarpJNI.ConnectionReader_getWriter(swigCPtr, this);
    return (cPtr == 0) ? null : new ConnectionWriter(cPtr, false);
  }

  public SWIGTYPE_p_Bytes readEnvelope() {
    return new SWIGTYPE_p_Bytes(yarpJNI.ConnectionReader_readEnvelope(swigCPtr, this), true);
  }

  public Portable getReference() {
    long cPtr = yarpJNI.ConnectionReader_getReference(swigCPtr, this);
    return (cPtr == 0) ? null : new Portable(cPtr, false);
  }

  public Contact getRemoteContact() {
    return new Contact(yarpJNI.ConnectionReader_getRemoteContact(swigCPtr, this), true);
  }

  public Contact getLocalContact() {
    return new Contact(yarpJNI.ConnectionReader_getLocalContact(swigCPtr, this), true);
  }

  public boolean isValid() {
    return yarpJNI.ConnectionReader_isValid(swigCPtr, this);
  }

  public boolean isActive() {
    return yarpJNI.ConnectionReader_isActive(swigCPtr, this);
  }

  public boolean isError() {
    return yarpJNI.ConnectionReader_isError(swigCPtr, this);
  }

  public void requestDrop() {
    yarpJNI.ConnectionReader_requestDrop(swigCPtr, this);
  }

  public SWIGTYPE_p_Searchable getConnectionModifiers() {
    return new SWIGTYPE_p_Searchable(yarpJNI.ConnectionReader_getConnectionModifiers(swigCPtr, this), false);
  }

  public boolean pushInt(int x) {
    return yarpJNI.ConnectionReader_pushInt(swigCPtr, this, x);
  }

  public boolean setSize(long len) {
    return yarpJNI.ConnectionReader_setSize(swigCPtr, this, len);
  }

  public static ConnectionReader createConnectionReader(SWIGTYPE_p_yarp__os__InputStream is) {
    long cPtr = yarpJNI.ConnectionReader_createConnectionReader(SWIGTYPE_p_yarp__os__InputStream.getCPtr(is));
    return (cPtr == 0) ? null : new ConnectionReader(cPtr, false);
  }

  public static boolean readFromStream(PortReader portable, SWIGTYPE_p_yarp__os__InputStream is) {
    return yarpJNI.ConnectionReader_readFromStream(PortReader.getCPtr(portable), portable, SWIGTYPE_p_yarp__os__InputStream.getCPtr(is));
  }

}
