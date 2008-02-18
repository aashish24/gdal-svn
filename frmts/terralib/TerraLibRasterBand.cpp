/*****************************************************************************
 * $Id: $
 *
 * Project:  TerraLib Raster Database schema support
 * Purpose:  Read/Write TerraLib Raster band (see TerraLib.org)
 * Author:   Ivan Lucena [ivan.lucena@pmldnet.com]
 *
 ******************************************************************************
 * Copyright (c) 2007, Ivan Lucena
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files ( the "Software" ),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/

#include "TerraLibRasterBand.h"

//  ----------------------------------------------------------------------------
//                                                          TerraLibRasterBand()
//  ----------------------------------------------------------------------------

TerraLibRasterBand::TerraLibRasterBand( TerraLibDataset *poDS, int nBand )
{
}

//  ----------------------------------------------------------------------------
//                                                         ~TerraLibRasterBand()
//  ----------------------------------------------------------------------------

TerraLibRasterBand::~TerraLibRasterBand()
{
}

//  ----------------------------------------------------------------------------
//                                                                  GetMinimum()
//  ----------------------------------------------------------------------------

double TerraLibRasterBand::GetMinimum( int *pbSuccess )
{
    return FALSE;
}

//  ----------------------------------------------------------------------------
//                                                                  GetMaximum()
//  ----------------------------------------------------------------------------

double TerraLibRasterBand::GetMaximum( int *pbSuccess )
{
    return FALSE;
}
    
//  ----------------------------------------------------------------------------
//                                                               GetColorTable()
//  ----------------------------------------------------------------------------

GDALColorTable *TerraLibRasterBand::GetColorTable()
{
    return NULL;
}

//  ----------------------------------------------------------------------------
//                                                      GetColorInterpretation()
//  ----------------------------------------------------------------------------

GDALColorInterp TerraLibRasterBand::GetColorInterpretation()
{
    return GCI_Undefined;
}

//  ----------------------------------------------------------------------------
//                                                                  IReadBlock()
//  ----------------------------------------------------------------------------

CPLErr TerraLibRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff, void *pImage )
{
    return CE_None;
}

//  ----------------------------------------------------------------------------
//                                                                 IWriteBlock()
//  ----------------------------------------------------------------------------

CPLErr TerraLibRasterBand::IWriteBlock( int nBlockXOff, int nBlockYOff, void *pImage )
{
    return CE_None;
}

//  ----------------------------------------------------------------------------
//                                                               SetColorTable()
//  ----------------------------------------------------------------------------

CPLErr TerraLibRasterBand::SetColorTable( GDALColorTable *poColorTable )
{
    return CE_None;
}
 
//  ----------------------------------------------------------------------------
//                                                               SetStatistics()
//  ----------------------------------------------------------------------------

CPLErr TerraLibRasterBand::SetStatistics( double dfMin, double dfMax, 
                                          double dfMean, double dfStdDev )
{
    return CE_None;
}

