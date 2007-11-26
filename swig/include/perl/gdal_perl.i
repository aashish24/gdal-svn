/******************************************************************************
 *
 * Project:  GDAL SWIG Interface declarations for Perl.
 * Purpose:  GDAL declarations.
 * Author:   Ari Jolma and Kevin Ruland
 *
 ******************************************************************************
 * Copyright (c) 2007, Ari Jolma and Kevin Ruland
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/

%init %{
  /* gdal_perl.i %init code */
  UseExceptions();
  if ( GDALGetDriverCount() == 0 ) {
    GDALAllRegister();
  }
%}

%inline %{
    typedef struct
    {
	SV *fct;
	SV *data;
    } SavedEnv;
    int callback_d_cp_vp(double d, const char *cp, void *vp)
    {
	int count, ret;
	SavedEnv *env_ptr = (SavedEnv *)vp;
	dSP;
	ENTER;
	SAVETMPS;
	PUSHMARK(SP);
	XPUSHs(sv_2mortal(newSVnv(d)));
	XPUSHs(sv_2mortal(newSVpv(cp, 0)));
	if (env_ptr->data)
	    XPUSHs(env_ptr->data);
	PUTBACK;
	count = call_sv(env_ptr->fct, G_SCALAR);
	SPAGAIN;
	if (count != 1)
	    croak("Big trouble\n");
	ret = POPi;
	PUTBACK;
	FREETMPS;
	LEAVE;
	return ret;
    }
%}

%include cpl_exceptions.i

%rename (GetMetadata) GetMetadata_Dict;
%ignore GetMetadata_List;

%import typemaps_perl.i

%import destroy.i

/*ALTERED_DESTROY(GDALRasterBandShadow, GDALc, delete_Band) !does not work! */
ALTERED_DESTROY(GDALColorTableShadow, GDALc, delete_ColorTable)
ALTERED_DESTROY(GDALConstShadow, GDALc, delete_Const)
ALTERED_DESTROY(GDALDatasetShadow, GDALc, delete_Dataset)
ALTERED_DESTROY(GDALDriverShadow, GDALc, delete_Driver)
ALTERED_DESTROY(GDAL_GCP, GDALc, delete_GCP)
ALTERED_DESTROY(GDALMajorObjectShadow, GDALc, delete_MajorObject)
ALTERED_DESTROY(GDALRasterAttributeTableShadow, GDALc, delete_RasterAttributeTable)

%rename (_GetDataTypeSize) GetDataTypeSize;
%rename (_DataTypeIsComplex) DataTypeIsComplex;

%rename (_GetDriver) GetDriver;

%rename (_Open) Open;
%newobject _Open;

%rename (_OpenShared) OpenShared;
%newobject _OpenShared;

%rename (_ComputeMedianCutPCT) ComputeMedianCutPCT;
%rename (_DitherRGB2PCT) DitherRGB2PCT;
%rename (_ReprojectImage) ReprojectImage;
%rename (_AutoCreateWarpedVRT) AutoCreateWarpedVRT;
%newobject _AutoCreateWarpedVRT;

%rename (_Create) Create;
%newobject _Create;

%rename (_GetRasterBand) GetRasterBand;
%rename (_AddBand) AddBand;

%rename (_GetPaletteInterpretation) GetPaletteInterpretation;

%rename (_SetColorEntry) SetColorEntry;

%rename (_GetUsageOfCol) GetUsageOfCol;
%rename (_GetColOfUsage) GetColOfUsage;
%rename (_GetTypeOfCol) GetTypeOfCol;
%rename (_CreateColumn) CreateColumn;

%perlcode %{
    use strict;
    use Carp;
    use Geo::GDAL::Const;
    use Geo::OGR;
    use Geo::OSR;
    our $VERSION = '0.21';
    use vars qw/
	%TYPE_STRING2INT %TYPE_INT2STRING
	%ACCESS_STRING2INT %ACCESS_INT2STRING
	%RESAMPLING_STRING2INT %RESAMPLING_INT2STRING
	%NODE_TYPE_STRING2INT %NODE_TYPE_INT2STRING
	/;
    for my $string (qw/Unknown Byte UInt16 Int16 UInt32 Int32 Float32 Float64 CInt16 CInt32 CFloat32 CFloat64/) {
	my $int = eval "\$Geo::GDAL::Constc::GDT_$string";
	$TYPE_STRING2INT{$string} = $int;
	$TYPE_INT2STRING{$int} = $string;
    }
    for my $string (qw/ReadOnly Update/) {
	my $int = eval "\$Geo::GDAL::Constc::GA_$string";
	$ACCESS_STRING2INT{$string} = $int;
	$ACCESS_INT2STRING{$int} = $string;
    }
    for my $string (qw/NearestNeighbour Bilinear Cubic CubicSpline/) {
	my $int = eval "\$Geo::GDAL::Constc::GRA_$string";
	$RESAMPLING_STRING2INT{$string} = $int;
	$RESAMPLING_INT2STRING{$int} = $string;
    }
    {
	my $int = 0;
	for my $string (qw/Element Text Attribute Comment Literal/) {
	    $NODE_TYPE_STRING2INT{$string} = $int;
	    $NODE_TYPE_INT2STRING{$int} = $string;
	    $int++;
	}
    }
    sub RELEASE_PARENTS {
    }
    sub NodeType {
	my $type = shift;
	return $NODE_TYPE_INT2STRING{$type} if $type =~ /^\d/;
	return $NODE_TYPE_STRING2INT{$type};
    }
    sub NodeData {
	my $node = shift;
	return (Geo::GDAL::NodeType($node->[0]), $node->[1]);
    }
    sub Children {
	my $node = shift;
	return @$node[2..$#$node];
    }
    sub Child {
	my($node, $child) = @_;
	return $node->[2+$child];
    }
    sub GetDataTypeSize {
	my $t = shift;
	$t = $TYPE_INT2STRING{$t} if exists $TYPE_INT2STRING{$t};
	return _GetDataTypeSize($t);
    }
    sub DataTypeIsComplex {
	my $t = shift;
	$t = $TYPE_INT2STRING{$t} if exists $TYPE_INT2STRING{$t};
	return _DataTypeIsComplex($t);
    }
    sub PackCharacter {
	my $t = shift;
	$t = $TYPE_INT2STRING{$t} if exists $TYPE_INT2STRING{$t};
	my $is_big_endian = unpack("h*", pack("s", 1)) =~ /01/; # from Programming Perl
	return 'C' if $t =~ /^Byte$/;
	return ($is_big_endian ? 'n': 'v') if $t =~ /^UInt16$/;
	return 's' if $t =~ /^Int16$/;
	return ($is_big_endian ? 'N' : 'V') if $t =~ /^UInt32$/;
	return 'l' if $t =~ /^Int32$/;
	return 'f' if $t =~ /^Float32$/;
	return 'd' if $t =~ /^Float64$/;
	croak "unsupported data type: $t";
    }
    sub GetDriver {
	my $driver = shift;
	return _GetDriver($driver) if $driver =~ /^\d/;
	return GetDriverByName($driver);
    }
    sub Open {
	my @p = @_;
	$p[1] = $ACCESS_STRING2INT{$p[1]} if $p[1] and exists $ACCESS_STRING2INT{$p[1]};
	return _Open(@p);
    }
    sub OpenShared {
	my @p = @_;
	$p[1] = $ACCESS_STRING2INT{$p[1]} if $p[1] and exists $ACCESS_STRING2INT{$p[1]};
	return _OpenShared(@p);
    }
    sub ComputeMedianCutPCT {
	my($red, $green, $blue, $num_colors, $colors, $callback, $callback_data) = @_;
	_ComputeMedianCutPCT($red, $green, $blue, $num_colors, $colors, $callback, $callback_data);
    }
    sub DitherRGB2PCT {
	my($red, $green, $blue, $target, $colors, $callback, $callback_data) = @_;
	_DitherRGB2PCT($red, $green, $blue, $target, $colors, $callback, $callback_data);
    }
    sub ReprojectImage {
	my @p = @_;
	$p[4] = $RESAMPLING_STRING2INT{$p[4]} if $p[4] and exists $RESAMPLING_STRING2INT{$p[4]};
	return _ReprojectImage(@p);
    }
    sub AutoCreateWarpedVRT {
	my @p = @_;
	$p[3] = $RESAMPLING_STRING2INT{$p[3]} if $p[3] and exists $RESAMPLING_STRING2INT{$p[3]};
	return _AutoCreateWarpedVRT(@p);
    }
    package Geo::GDAL::MajorObject;
    use strict;
    sub Description {
	my($self, $desc) = @_;
	SetDescription($self, $desc) if defined $desc;
	GetDescription($self) if defined wantarray;
    }
    sub Metadata {
	my $self = shift;
	my $metadata = shift if ref $_[0];
	my $domain = shift;
	$domain = '' unless defined $domain;
	SetMetadata($self, $metadata, $domain) if defined $metadata;
	GetMetadata($self, $domain) if defined wantarray;
    }
    package Geo::GDAL::Driver;
    use vars qw/@CAPABILITIES/;
    use strict;
    @CAPABILITIES = ('Create', 'CreateCopy');
    sub Name {
	my $self = shift;
	return $self->{ShortName};
    }
    sub Capabilities {
	my $self = shift;
	return @CAPABILITIES unless shift;
	my $h = $self->GetMetadata;
	my @cap;
	for my $cap (@CAPABILITIES) {
	    push @cap, $cap if $h->{'DCAP_'.uc($cap)} eq 'YES';
	}
	return @cap;
    }
    sub TestCapability {
	my($self, $cap) = @_;
	my $h = $self->GetMetadata;
	return $h->{'DCAP_'.uc($cap)} eq 'YES' ? 1 : undef;
    }
    sub Extension {
	my $self = shift;
	my $h = $self->GetMetadata;
	return $h->{DMD_EXTENSION};
    }
    sub MIMEType {
	my $self = shift;
	my $h = $self->GetMetadata;
	return $h->{DMD_MIMETYPE};
    }
    sub CreationOptionList {
	my $self = shift;
	my @options;
	my $h = $self->GetMetadata->{DMD_CREATIONOPTIONLIST};
	if ($h) {
	    $h = Geo::GDAL::ParseXMLString($h);
	    my($type, $value) = Geo::GDAL::NodeData($h);
	    if ($value eq 'CreationOptionList') {
		for my $o (Geo::GDAL::Children($h)) {
		    my %option;
		    for my $a (Geo::GDAL::Children($o)) {
			my(undef, $key) = Geo::GDAL::NodeData($a);
			my(undef, $value) = Geo::GDAL::NodeData(Geo::GDAL::Child($a, 0));
			if ($key eq 'Value') {
			    push @{$option{$key}}, $value;
			} else {
			    $option{$key} = $value;
			}
		    }
		    push @options, \%option;
		}
	    }
	}
	return @options;
    }
    sub CreationDataTypes {
	my $self = shift;
	my $h = $self->GetMetadata;
	return split /\s+/, $h->{DMD_CREATIONDATATYPES};
    }
    sub CreateDataset {
	my @p = @_;
	$p[5] = $Geo::GDAL::TYPE_STRING2INT{$p[5]} if $p[5] and exists $Geo::GDAL::TYPE_STRING2INT{$p[5]};
	return _Create(@p);
    }
    *Create = *CreateDataset;
    *CopyDataset = *CreateCopy;
    package Geo::GDAL::Dataset;
    use strict;
    use vars qw/%BANDS/;
    sub Open {
	return Geo::GDAL::Open(@_);
    }
    sub OpenShared {
	return Geo::GDAL::OpenShared(@_);
    }
    sub Size {
	my $self = shift;
	return ($self->{RasterXSize}, $self->{RasterYSize});
    }
    sub GetRasterBand {
	my($self, $index) = @_;
	my $band = _GetRasterBand($self, $index);
	$BANDS{tied(%{$band})} = $self;
	return $band;
    }
    sub AddBand {
	my @p = @_;
	$p[1] = $Geo::GDAL::TYPE_STRING2INT{$p[1]} if $p[1] and exists $Geo::GDAL::TYPE_STRING2INT{$p[1]};
	return _AddBand(@p);
    }
    sub Projection {
	my($self, $proj) = @_;
	SetProjection($self, $proj) if defined $proj;
	GetProjection($self) if defined wantarray;
    }
    sub GeoTransform {
	my $self = shift;
	SetGeoTransform($self, \@_) if @_ > 0;
	return unless defined wantarray;
	my $t = GetGeoTransform($self);
	return @$t;
    }
    sub GCPs {
	my $self = shift;
	if (@_ > 0) {
	    my $proj = pop @_;
	    SetGCPs($self, \@_, $proj);
	}
	return unless defined wantarray;
	my $proj = GetGCPProjection($self);
	my $GCPs = GetGCPs($self);
	return (@$GCPs, $proj);
    }
    package Geo::GDAL::Band;
    use strict;
    use vars qw/
	%COLOR_INTERPRETATION_STRING2INT %COLOR_INTERPRETATION_INT2STRING
	/;
    for my $string (qw/Undefined GrayIndex PaletteIndex RedBand GreenBand BlueBand AlphaBand 
		    HueBand SaturationBand LightnessBand CyanBand MagentaBand YellowBand BlackBand/) {
	my $int = eval "\$Geo::GDAL::Constc::GCI_$string";
	$COLOR_INTERPRETATION_STRING2INT{$string} = $int;
	$COLOR_INTERPRETATION_INT2STRING{$int} = $string;
    }
    sub DESTROY {
	my $self;
	if ($_[0]->isa('SCALAR')) {
	    $self = $_[0];
	} else {
	    return unless $_[0]->isa('HASH');
	    $self = tied(%{$_[0]});
	    return unless defined $self;
	}
	delete $ITERATORS{$self};
	if (exists $OWNER{$self}) {
	    delete $OWNER{$self};
	}
	$self->RELEASE_PARENTS();
    }
    sub RELEASE_PARENTS {
	my $self = shift;
	delete $Geo::GDAL::Dataset::BANDS{$self};
    }
    sub Size {
	my $self = shift;
	return ($self->{XSize}, $self->{YSize});
    }
    sub DataType {
	my $self = shift;
	return $Geo::GDAL::TYPE_INT2STRING{$self->{DataType}};
    }
    sub NoDataValue {
	my $self = shift;
	SetNoDataValue($self, $_[0]) if @_ > 0;
	GetNoDataValue($self);
    }
    sub ReadTile {
	my($self, $xoff, $yoff, $xsize, $ysize) = @_;
	$xoff = 0 unless defined $xoff;
	$yoff = 0 unless defined $yoff;
	$xsize = $self->{XSize} - $xoff unless defined $xsize;
	$ysize = $self->{YSize} - $yoff unless defined $ysize;
	my $buf = $self->ReadRaster($xoff, $yoff, $xsize, $ysize);
	my $pc = Geo::GDAL::PackCharacter($self->{DataType});
	my $w = $xsize * Geo::GDAL::GetDataTypeSize($self->{DataType})/8;
	my $offset = 0;
	my @data;
	for (0..$ysize-1) {
	    my $sub = substr($buf, $offset, $w);
	    my @d = unpack($pc."[$xsize]", $sub);
	    push @data, \@d;
	    $offset += $w;
	}
	return \@data;
    }
    sub WriteTile {
	my($self, $data, $xoff, $yoff) = @_;
	$xoff = 0 unless defined $xoff;
	$yoff = 0 unless defined $yoff;
	my $xsize = @{$data->[0]};
	$xsize = $self->{XSize} - $xoff if $xsize > $self->{XSize} - $xoff;
	my $ysize = @{$data};
	$ysize = $self->{YSize} - $yoff if $ysize > $self->{YSize} - $yoff;
	my $pc = Geo::GDAL::PackCharacter($self->{DataType});
	my $w = $xsize * Geo::GDAL::GetDataTypeSize($self->{DataType})/8;
	for my $i (0..$ysize-1) {
	    my $scanline = pack($pc."[$xsize]", @{$data->[$i]});
	    $self->WriteRaster( $xoff, $yoff+$i, $xsize, 1, $scanline );
	}
    }
    sub ColorInterpretation {
	my($self, $ci) = @_;
	if ($ci) {
	    $ci = $COLOR_INTERPRETATION_STRING2INT{$ci} if exists $COLOR_INTERPRETATION_STRING2INT{$ci};
	    SetRasterColorInterpretation($self, $ci);
	    return $ci;
	} else {
	    return $COLOR_INTERPRETATION_INT2STRING{GetRasterColorInterpretation($self)};
	}
    }
    sub ColorTable {
	my $self = shift;
	SetRasterColorTable($self, $_[0]) if @_;
	return unless defined wantarray;
	GetRasterColorTable($self);
    }
    sub CategoryNames {
	my $self = shift;
	SetRasterCategoryNames($self, \@_) if @_;
	return unless defined wantarray;
	my $n = GetRasterCategoryNames($self);
	return @$n;
    }
    sub AttributeTable {
	my $self = shift;
	SetDefaultRAT($self, $_[0]) if @_;
	return unless defined wantarray;
	my $r = GetDefaultRAT($self);
	$Geo::GDAL::RasterAttributeTable::BANDS{$r} = $self;
	return $r;
    }
    package Geo::GDAL::ColorTable;
    use strict;
    use vars qw/
	%PALETTE_INTERPRETATION_STRING2INT %PALETTE_INTERPRETATION_INT2STRING
	/;
    for my $string (qw/Gray RGB CMYK HLS/) {
	my $int = eval "\$Geo::GDAL::Constc::GPI_$string";
	$PALETTE_INTERPRETATION_STRING2INT{$string} = $int;
	$PALETTE_INTERPRETATION_INT2STRING{$int} = $string;
    }
    sub create {
	my($pkg, $pi) = @_;
	$pi = $PALETTE_INTERPRETATION_STRING2INT{$pi} if defined $pi and exists $PALETTE_INTERPRETATION_STRING2INT{$pi};
	my $self = Geo::GDALc::new_ColorTable($pi);
	bless $self, $pkg if defined($self);
    }
    sub GetPaletteInterpretation {
	my $self = shift;
	return $PALETTE_INTERPRETATION_INT2STRING{GetPaletteInterpretation($self)};
    }
    sub SetColorEntry {
	@_ == 3 ? _SetColorEntry(@_) : _SetColorEntry(@_[0..1], [@_[2..5]]);
    }
    sub ColorEntry {
	my $self = shift;
	my $index = shift;
	SetColorEntry($self, $index, @_) if @_ > 0;
	GetColorEntry($self, $index) if defined wantarray;
    }
    sub ColorTable {
	my $self = shift;
	my @table;
	if (@_) {
	    my $index = 0;
	    for my $color (@_) {
		push @table, [ColorEntry($self, $index, @$color)];
		$index++;
	    }
	} else {
	    for (my $index = 0; $index < GetCount($self); $index++) {
		push @table, [ColorEntry($self, $index)];
	    }
	}
	return @table;
    }
    package Geo::GDAL::RasterAttributeTable;
    use strict;
    use vars qw/ %BANDS
	%FIELD_TYPE_STRING2INT %FIELD_TYPE_INT2STRING
	%FIELD_USAGE_STRING2INT %FIELD_USAGE_INT2STRING
	/;
    for my $string (qw/Integer Real String/) {
	my $int = eval "\$Geo::GDAL::Constc::GFT_$string";
	$FIELD_TYPE_STRING2INT{$string} = $int;
	$FIELD_TYPE_INT2STRING{$int} = $string;
    }
    for my $string (qw/Generic PixelCount Name Min Max MinMax 
		    Red Green Blue Alpha RedMin 
		    GreenMin BlueMin AlphaMin RedMax GreenMax BlueMax AlphaMax 
		    MaxCount/) {
	my $int = eval "\$Geo::GDAL::Constc::GFU_$string";
	$FIELD_USAGE_STRING2INT{$string} = $int;
	$FIELD_USAGE_INT2STRING{$int} = $string;
    }
    sub FieldTypes {
	return keys %FIELD_TYPE_STRING2INT;
    }
    sub FieldUsages {
	return keys %FIELD_USAGE_STRING2INT;
    }
    sub RELEASE_PARENTS {
	my $self = shift;
	delete $BANDS{$self};
    }
    sub GetUsageOfCol {
	my($self, $col) = @_;
	$FIELD_USAGE_INT2STRING{_GetUsageOfCol($self, $col)};
    }
    sub GetColOfUsage {
	my($self, $usage) = @_;
	_GetColOfUsage($self, $FIELD_USAGE_STRING2INT{$usage});
    }
    sub GetTypeOfCol {
	my($self, $col) = @_;
	$FIELD_TYPE_INT2STRING{_GetTypeOfCol($self, $col)};
    }
    sub CreateColumn {
	my($self, $name, $type, $usage) = @_;
	_CreateColumn($self, $name, $FIELD_TYPE_STRING2INT{$type}, $FIELD_USAGE_STRING2INT{$usage});
    }
    sub Value {
	my($self, $row, $column) = @_;
	SetValueAsString($self, $row, $column, $_[3]) if defined $_[3];
	return unless defined wantarray;
	GetValueAsString($self, $row, $column);
    }

 %}
