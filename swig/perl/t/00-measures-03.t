use strict;
use warnings;
use bytes;
use v5.10;
use Test::More qw(no_plan);
BEGIN { use_ok('Geo::GDAL') };

# test measured geometries in shape driver

my @data = (
    PointM => 'POINT M (1 2 3)',
    LineStringM => 'LINESTRING M (1 2 3)',
    PointZM => 'POINT ZM (1 2 3 4)',
    LineStringZM => 'LINESTRING ZM (1 2 3 4)',
    PolygonM => 'POLYGON M ((1 2 3))',
    MultiPointM => 'MULTIPOINT M ((1 2 3))',
#    MultiLineStringM => 'MULTILINESTRING M ((1 2 3))',
#    MultiPolygonM => 'MULTIPOLYGON M (((1 2 3)))',
#    CircularStringM => 'CIRCULARSTRING M (1 2 3,1 2 3,1 2 3)',
#    CompoundCurveM => 'COMPOUNDCURVE M ((0 1 2,2 3 4))',
#    CurvePolygonM => 'CURVEPOLYGON M ((0 0 1,0 1 1,1 1 1,1 0 1,0 0 1))',
#    MultiCurveM => 'MULTICURVE M (CIRCULARSTRING M (0 0 2,1 0 2,0 0 2),(0 0 2,1 1 2))',
#    MultiSurfaceM => 'MULTISURFACE M (((1 2 3)))',

    PolygonZM => 'POLYGON ZM ((1 2 3 4))',
    MultiPointZM => 'MULTIPOINT ZM ((1 2 3 4))',
#    MultiLineStringZM => 'MULTILINESTRING ZM ((1 2 3 4))',
#    MultiPolygonZM => 'MULTIPOLYGON ZM (((1 2 3 4)))',
#    CircularStringZM => 'CIRCULARSTRING ZM (1 2 3 4,1 2 3 4,1 2 3 4)',
#    CompoundCurveZM => 'COMPOUNDCURVE ZM ((0 1 2 3,2 3 4 5))',
#    CurvePolygonZM => 'CURVEPOLYGON ZM ((0 0 1 2,0 1 1 2,1 1 1 2,1 0 1 2,0 0 1 2))',
#    MultiCurveZM => 'MULTICURVE ZM (CIRCULARSTRING ZM (0 0 2 3,1 0 2 3,0 0 2 3),(0 0 2 3,1 1 2 3))',
#    MultiSurfaceZM => 'MULTISURFACE ZM (((1 2 3 4)))',
);

my $driver = 'ESRI Shapefile';

for (my $i = 0; $i < @data; $i+=2) {
    my $type = $data[$i];
    #next unless $type eq 'CompoundCurveM';
    my $l = Geo::OGR::Driver($driver)->Create('/vsimem/test.shp')->CreateLayer(GeometryType => $type);
    eval {
        my $g = Geo::OGR::Geometry->new(WKT => $data[$i+1]);
        $l->InsertFeature({Geometry => {WKT => $data[$i+1]}});
    };
    if ($@) {
        my @e = split /\n/, $@;
        $e[0] =~ s/\. at .*/./;
        ok(0, "$driver, insert feature: $type => $data[$i+1] ($e[0])");
        next;
    }
    $l->ResetReading;
    while (my $f = $l->GetNextFeature()) {
        my $g = $f->GetGeometryRef;
        my $t = $g->GeometryType;
        ok($t eq $type, "$driver retrieve feature: expected: $type, got: $t");
        my $wkt = $g->As(Format => 'ISO WKT');
        ok($wkt eq $data[$i+1], "$driver retrieve feature: $type, expected: $data[$i+1] got: $wkt");
    }
}
