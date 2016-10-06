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
package asap.eyepiengine.planunit;

import java.lang.Exception;

/**
 * Thrown whenever a EyePiUnit fails during play
 * @author Dennis
 */
public class EPUPlayException extends Exception
{
    private static final long serialVersionUID = 14423L;
    private final EyePiUnit wu;
    
    public EPUPlayException(String str, EyePiUnit wu, Exception ex)
    {
        this(str,wu);
        initCause(ex);
    }
    
    public EPUPlayException(String str, EyePiUnit wu)
    {
        super(str);
        this.wu = wu;
    }
    
    public final EyePiUnit getEyePiUnit()
    {
        return wu;
    }
}