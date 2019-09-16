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
xmin = -5; 
ymin = -5;
xmax =  5;
ymax =  5;

Polygon[] polygons = { Polygon( 5, (0,0), 1, 0, red ),
                       Polygon( 5, (3,3), 1.5, 45, red ),
                       Polygon( 6, (-2,1), 0.8, 90, red ),
                       Polygon( 3, (3,-2), 0.9, 90, blue ),
                       Polygon( 4, (-3,-2), 0.6, 0, blue ),
                       Polygon( 300, (-2,3), 0.6, 0, green ),
                       Polygon( 300, (-1,-3), 0.6, 0, green ),
                     };

for (int i=0; i<polygons.length; ++i) {
    fill(shift(polygons[i].origin)*scale(polygons[i].radius)*rotate(polygons[i].angle)*polygon(polygons[i].n), polygons[i].color);    
}

draw(box((xmin,ymin), (xmax,ymax)), miterjoin+0.2cm);
xaxis(xmin-1, xmax+1, invisible);
yaxis(ymin-1, ymax+1, invisible);
