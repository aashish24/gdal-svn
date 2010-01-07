/******************************************************************************
 * $Id$
 *
 * Project:  VFK Reader - Property definition
 * Purpose:  Implements VFKProperty class.
 * Author:   Martin Landa, landa.martin gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2009-2010, Martin Landa <landa.martin gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ****************************************************************************/

#include "vfkreader.h"
#include "vfkreaderp.h"

#include "cpl_conv.h"
#include "cpl_error.h"

/*!
  \brief Set VFK property (null)
*/
VFKProperty::VFKProperty()
{
    b_isNull = TRUE;
}

/*!
  \brief Set VFK property (integer)
*/
VFKProperty::VFKProperty(int iValue)
{
    b_isNull = FALSE;
    i = iValue;
}

/*!
  \brief Set VFK property (double)
*/
VFKProperty::VFKProperty(double dValue)
{
    b_isNull = FALSE;
    d = dValue;
}

/*!
  \brief Set VFK property (string)
*/
VFKProperty::VFKProperty(const char *pszValue)
{
    b_isNull = FALSE;
    s = CPLStrdup(pszValue);
}
