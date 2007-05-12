%extend OGRLayerShadow {
// File: ogrlayer_8cpp.xml
%feature("docstring")  CPL_CVSID "CPL_CVSID(\"$Id: ogrlayer.cpp 10645
2007-01-18 02:22:39Z warmerdam $\") ";

%feature("docstring")  Reference "int OGR_L_Reference(OGRLayerH
hLayer) ";

%feature("docstring")  Dereference "int OGR_L_Dereference(OGRLayerH
hLayer) ";

%feature("docstring")  GetRefCount "int OGR_L_GetRefCount(OGRLayerH
hLayer) ";

%feature("docstring")  GetFeatureCount "int
OGR_L_GetFeatureCount(OGRLayerH hLayer, int bForce) ";

%feature("docstring")  GetExtent "OGRErr OGR_L_GetExtent(OGRLayerH
hLayer, OGREnvelope *psExtent, int bForce)

Fetch the extent of this layer.

Returns the extent (MBR) of the data in the layer. If bForce is FALSE,
and it would be expensive to establish the extent then OGRERR_FAILURE
will be returned indicating that the extent isn't know. If bForce is
TRUE then some implementations will actually scan the entire layer
once to compute the MBR of all the features in the layer.

The returned extent does not take the spatial filter into account. If
a spatial filter was previously set then it should be ignored but some
implementations may be unable to do that, so it is safer to call
OGR_L_GetExtent() without setting a spatial filter.

Layers without any geometry may return OGRERR_FAILURE just indicating
that no meaningful extents could be collected.

This function is the same as the C++ method OGRLayer::GetExtent().

Parameters:
-----------

hLayer:  handle to the layer from which to get extent.

psExtent:  the structure in which the extent value will be returned.

bForce:  Flag indicating whether the extent should be computed even if
it is expensive.

OGRERR_NONE on success, OGRERR_FAILURE if extent not known. ";

%feature("docstring")  SetAttributeFilter "OGRErr
OGR_L_SetAttributeFilter(OGRLayerH hLayer, const char *pszQuery)

Set a new attribute query.

This function sets the attribute query string to be used when fetching
features via the OGR_L_GetNextFeature() function. Only features for
which the query evaluates as true will be returned.

The query string should be in the format of an SQL WHERE clause. For
instance \"population > 1000000 and population < 5000000\" where
population is an attribute in the layer. The query format is a
restricted form of SQL WHERE clause as defined
\"eq_format=restricted_where\" about half way through this document:

http://ogdi.sourceforge.net/prop/6.2.CapabilitiesMetadata.html

Note that installing a query string will generally result in resetting
the current reading position (ala OGR_L_ResetReading()).

This function is the same as the C++ method
OGRLayer::SetAttributeFilter().

Parameters:
-----------

hLayer:  handle to the layer on which attribute query will be
executed.

pszQuery:  query in restricted SQL WHERE format, or NULL to clear the
current query.

OGRERR_NONE if successfully installed, or an error code if the query
expression is in error, or some other failure occurs. ";

%feature("docstring")  GetFeature "OGRFeatureH
OGR_L_GetFeature(OGRLayerH hLayer, long nFeatureId)

Fetch a feature by it's identifier.

This function will attempt to read the identified feature. The nFID
value cannot be OGRNullFID. Success or failure of this operation is
unaffected by the spatial or attribute filters.

If this function returns a non-NULL feature, it is guaranteed that
it's feature id ( OGR_F_GetFID()) will be the same as nFID.

Use OGR_L_TestCapability(OLCRandomRead) to establish if this layer
supports efficient random access reading via OGR_L_GetFeature();
however, the call should always work if the feature exists as a
fallback implementation just scans all the features in the layer
looking for the desired feature.

Sequential reads are generally considered interrupted by a
OGR_L_GetFeature() call.

This function is the same as the C++ method OGRLayer::GetFeature( ).

Parameters:
-----------

hLayer:  handle to the layer that owned the feature.

nFeatureId:  the feature id of the feature to read.

an handle to a feature now owned by the caller, or NULL on failure. ";

%feature("docstring")  SetNextByIndex "OGRErr
OGR_L_SetNextByIndex(OGRLayerH hLayer, long nIndex) ";

%feature("docstring")  GetNextFeature "OGRFeatureH
OGR_L_GetNextFeature(OGRLayerH hLayer)

Fetch the next available feature from this layer. The returned feature
becomes the responsiblity of the caller to delete. It is critical that
all features associated with an OGRLayer (more specifically an
OGRFeatureDefn) be deleted before that layer/datasource is deleted.

Only features matching the current spatial filter (set with
SetSpatialFilter()) will be returned.

This function implements sequential access to the features of a layer.
The OGR_L_ResetReading() function can be used to start at the
beginning again. Random reading, writing and spatial filtering will be
added to the OGRLayer in the future.

This function is the same as the C++ method
OGRLayer::GetNextFeature().

Parameters:
-----------

hLayer:  handle to the layer from which feature are read.

an handle to a feature, or NULL if no more features are available. ";

%feature("docstring")  SetFeature "OGRErr OGR_L_SetFeature(OGRLayerH
hLayer, OGRFeatureH hFeat)

Rewrite an existing feature.

This function will write a feature to the layer, based on the feature
id within the OGRFeature.

Use OGR_L_TestCapability(OLCRandomWrite) to establish if this layer
supports random access writing via OGR_L_SetFeature().

This function is the same as the C++ method OGRLayer::SetFeature().

Parameters:
-----------

hLayer:  handle to the layer to write the feature.

hFeat:  the feature to write.

OGRERR_NONE if the operation works, otherwise an appropriate error
code. ";

%feature("docstring")  CreateFeature "OGRErr
OGR_L_CreateFeature(OGRLayerH hLayer, OGRFeatureH hFeat)

Create and write a new feature within a layer.

The passed feature is written to the layer as a new feature, rather
than overwriting an existing one. If the feature has a feature id
other than OGRNullFID, then the native implementation may use that as
the feature id of the new feature, but not necessarily. Upon
successful return the passed feature will have been updated with the
new feature id.

This function is the same as the C++ method OGRLayer::CreateFeature().

Parameters:
-----------

hLayer:  handle to the layer to write the feature to.

hFeat:  the handle of the feature to write to disk.

OGRERR_NONE on success. ";

%feature("docstring")  CreateField "OGRErr
OGR_L_CreateField(OGRLayerH hLayer, OGRFieldDefnH hField, int
bApproxOK)

Create a new field on a layer. You must use this to create new fields
on a real layer. Internally the OGRFeatureDefn for the layer will be
updated to reflect the new field. Applications should never modify the
OGRFeatureDefn used by a layer directly.

This function is the same as the C++ method OGRLayer::CreateField().

Parameters:
-----------

hLayer:  handle to the layer to write the field definition.

hField:  handle of the field definition to write to disk.

bApproxOK:  If TRUE, the field may be created in a slightly different
form depending on the limitations of the format driver.

OGRERR_NONE on success. ";

%feature("docstring")  StartTransaction "OGRErr
OGR_L_StartTransaction(OGRLayerH hLayer)

What does this do?.

This function is the same as the C++ method
OGRLayer::StartTransaction().

Parameters:
-----------

hLayer:  handle to the layer ?

OGRERR_NONE on success. ";

%feature("docstring")  CommitTransaction "OGRErr
OGR_L_CommitTransaction(OGRLayerH hLayer)

What does this do?.

This function is the same as the C++ method
OGRLayer::CommitTransaction().

Parameters:
-----------

hLayer:  handle to the layer?

OGRERR_NONE on success. ";

%feature("docstring")  RollbackTransaction "OGRErr
OGR_L_RollbackTransaction(OGRLayerH hLayer)

What does this do?.

This function is the same as the C++ method
OGRLayer::RollbackTransaction().

Parameters:
-----------

hLayer:  handle to the layer?

OGRERR_NONE on success. ";

%feature("docstring")  GetLayerDefn "OGRFeatureDefnH
OGR_L_GetLayerDefn(OGRLayerH hLayer)

Fetch the schema information for this layer.

The returned handle to the OGRFeatureDefn is owned by the OGRLayer,
and should not be modified or freed by the application. It
encapsulates the attribute schema of the features of the layer.

This function is the same as the C++ method OGRLayer::GetLayerDefn().

Parameters:
-----------

hLayer:  handle to the layer to get the schema information.

an handle to the feature definition. ";

%feature("docstring")  GetSpatialRef "OGRSpatialReferenceH
OGR_L_GetSpatialRef(OGRLayerH hLayer)

Fetch the spatial reference system for this layer.

The returned object is owned by the OGRLayer and should not be
modified or freed by the application.

This function is the same as the C++ method OGRLayer::GetSpatialRef().

Parameters:
-----------

hLayer:  handle to the layer to get the spatial reference from.

spatial reference, or NULL if there isn't one. ";

%feature("docstring")  TestCapability "int
OGR_L_TestCapability(OGRLayerH hLayer, const char *pszCap)

Test if this layer supported the named capability.

The capability codes that can be tested are represented as strings,
but defined constants exists to ensure correct spelling. Specific
layer types may implement class specific capabilities, but this can't
generally be discovered by the caller.

OLCRandomRead / \"RandomRead\": TRUE if the OGR_L_GetFeature()
function works for this layer.

OLCSequentialWrite / \"SequentialWrite\": TRUE if the
OGR_L_CreateFeature() function works for this layer. Note this means
that this particular layer is writable. The same OGRLayer class may
returned FALSE for other layer instances that are effectively read-
only.

OLCRandomWrite / \"RandomWrite\": TRUE if the OGR_L_SetFeature()
function is operational on this layer. Note this means that this
particular layer is writable. The same OGRLayer class may returned
FALSE for other layer instances that are effectively read-only.

OLCFastSpatialFilter / \"FastSpatialFilter\": TRUE if this layer
implements spatial filtering efficiently. Layers that effectively read
all features, and test them with the OGRFeature intersection methods
should return FALSE. This can be used as a clue by the application
whether it should build and maintain it's own spatial index for
features in this layer.

OLCFastFeatureCount / \"FastFeatureCount\": TRUE if this layer can
return a feature count (via OGR_L_GetFeatureCount()) efficiently ...
ie. without counting the features. In some cases this will return TRUE
until a spatial filter is installed after which it will return FALSE.

OLCFastGetExtent / \"FastGetExtent\": TRUE if this layer can return
its data extent (via OGR_L_GetExtent()) efficiently ... ie. without
scanning all the features. In some cases this will return TRUE until a
spatial filter is installed after which it will return FALSE.

This function is the same as the C++ method
OGRLayer::TestCapability().

Parameters:
-----------

hLayer:  handle to the layer to get the capability from.

pszCap:  the name of the capability to test.

TRUE if the layer has the requested capability, or FALSE otherwise.
OGRLayers will return FALSE for any unrecognised capabilities. ";

%feature("docstring")  GetSpatialFilter "OGRGeometryH
OGR_L_GetSpatialFilter(OGRLayerH hLayer)

This function returns the current spatial filter for this layer.

The returned pointer is to an internally owned object, and should not
be altered or deleted by the caller.

This function is the same as the C++ method
OGRLayer::GetSpatialFilter().

Parameters:
-----------

hLayer:  handle to the layer to get the spatial filter from.

an handle to the spatial filter geometry. ";

%feature("docstring")  SetSpatialFilter "void
OGR_L_SetSpatialFilter(OGRLayerH hLayer, OGRGeometryH hGeom)

Set a new spatial filter.

This function set the geometry to be used as a spatial filter when
fetching features via the OGR_L_GetNextFeature() function. Only
features that geometrically intersect the filter geometry will be
returned.

Currently this test is may be inaccurately implemented, but it is
guaranteed that all features who's envelope (as returned by
OGR_G_GetEnvelope()) overlaps the envelope of the spatial filter will
be returned. This can result in more shapes being returned that should
strictly be the case.

This function makes an internal copy of the passed geometry. The
passed geometry remains the responsibility of the caller, and may be
safely destroyed.

For the time being the passed filter geometry should be in the same
SRS as the layer (as returned by OGR_L_GetSpatialRef()). In the future
this may be generalized.

This function is the same as the C++ method
OGRLayer::SetSpatialFilter.

Parameters:
-----------

hLayer:  handle to the layer on which to set the spatial filter.

hGeom:  handle to the geometry to use as a filtering region. NULL may
be passed indicating that the current spatial filter should be
cleared, but no new one instituted. ";

%feature("docstring")  SetSpatialFilterRect "void
OGR_L_SetSpatialFilterRect(OGRLayerH hLayer, double dfMinX, double
dfMinY, double dfMaxX, double dfMaxY) ";

%feature("docstring")  ResetReading "void
OGR_L_ResetReading(OGRLayerH hLayer)

Reset feature reading to start on the first feature. This affects
GetNextFeature().

This function is the same as the C++ method OGRLayer::ResetReading().

Parameters:
-----------

hLayer:  handle to the layer on which features are read. ";

%feature("docstring")  SyncToDisk "OGRErr OGR_L_SyncToDisk(OGRLayerH
hDS) ";

%feature("docstring")  DeleteFeature "OGRErr
OGR_L_DeleteFeature(OGRLayerH hDS, long nFID)

Delete feature from layer.

The feature with the indicated feature id is deleted from the layer if
supported by the driver. Most drivers do not support feature deletion,
and will return OGRERR_UNSUPPORTED_OPERATION. The
OGR_L_TestCapability() function may be called with OLCDeleteFeature to
check if the driver supports feature deletion.

This method is the same as the C++ method OGRLayer::DeleteFeature().

Parameters:
-----------

poFeature:  the feature to write to disk.

OGRERR_NONE on success. ";

%feature("docstring")  GetFeaturesRead "GIntBig
OGR_L_GetFeaturesRead(OGRLayerH hLayer) ";

%feature("docstring")  GetFIDColumn "const char*
OGR_L_GetFIDColumn(OGRLayerH hLayer) ";

%feature("docstring")  GetGeometryColumn "const char*
OGR_L_GetGeometryColumn(OGRLayerH hLayer) ";

}