/**********************************************************************
 * $Id$
 *
 * Project:  CPL - Common Portability Library
 * Purpose:  Implementation of quadtree building and searching functions.
 *           Derived from shapelib and mapserver implementations
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999, Frank Warmerdam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#ifndef _CPL_QUAD_TREE_H_INCLUDED
#define _CPL_QUAD_TREE_H_INCLUDED

#include "cpl_port.h"

CPL_C_START

/* Types */

typedef struct {
  double minx, miny, maxx, maxy;
} CPLRectObj;

typedef struct _CPLQuadTree CPLQuadTree;

typedef void (*CPLQuadTreeGetBoundsFunc)(const void* feature, CPLRectObj* pBounds);

/* Functions */

CPLQuadTree CPL_DLL  *CPL_CreateTree(int numfeatures, void** features,
                                     CPLQuadTreeGetBoundsFunc get_bound_func,
                                     CPLRectObj* pGlobalBounds, int maxdepth);
void        CPL_DLL   CPL_DestroyTree(CPLQuadTree *tree);
void        CPL_DLL **CPL_SearchTree(CPLQuadTree *tree, CPLRectObj* pAoi, int* pnFeatureCount);
void        CPL_DLL   CPL_TreeTrim(CPLQuadTree *tree);

CPL_C_END

#endif
