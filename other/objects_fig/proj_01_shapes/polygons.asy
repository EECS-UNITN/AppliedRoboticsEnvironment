unitsize(1cm);

import graph;

struct Polygon {
    int  n;
    pair origin;
    real radius;
    real angle;
    pen  color;
    
    void operator init(int n, pair origin, real radius, real angle, pen color) {
        this.n      = n;
        this.origin = origin;
        this.radius = radius;
        this.angle  = angle;
        this.color  = color;
    }
};


real xmin, xmax, ymin, ymax;
xmin = -1; 
ymin = -1;
xmax =  1;
ymax =  1;

Polygon[] polygons = { Polygon( 3, (0,0), 2.0, 0, red ) };

for (int i=0; i<polygons.length; ++i) {
    fill(shift(polygons[i].origin)*scale(polygons[i].radius)*rotate(polygons[i].angle)*polygon(polygons[i].n), polygons[i].color);
}

xaxis(xmin-1, xmax+1, invisible);
yaxis(ymin-1, ymax+1, invisible);
