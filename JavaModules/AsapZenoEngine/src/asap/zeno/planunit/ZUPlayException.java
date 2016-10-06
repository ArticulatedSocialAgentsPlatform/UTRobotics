/*******************************************************************************
 * Copyright (C) 2009 Human Media Interaction, University of Twente, the Netherlands
 * 
 * This file is part of the Elckerlyc BML realizer.
 * 
 * Elckerlyc is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Elckerlyc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Elckerlyc.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
package asap.zeno.planunit;


/**
 * Thrown whenever a NaoUnit fails during play
 * @author Daniel
 */
public class ZUPlayException extends Exception
{
    private static final long serialVersionUID = 1423L;
    private final ZenoUnit zu;
    
    public ZUPlayException(String str, ZenoUnit zu, Exception ex)
    {
        this(str,zu);
        initCause(ex);
    }
    
    public ZUPlayException(String str, ZenoUnit zu)
    {
        super(str);
        this.zu = zu;
    }
    
    public final ZenoUnit getZenoUnit()
    {
        return zu;
    }
}