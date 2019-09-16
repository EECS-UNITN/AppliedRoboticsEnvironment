unitsize(1mm);

// Requested page boundaries.
real x_max = 210;
real y_max = 297;

import settings;
settings.render = 16;

import graph;

real x_c = x_max*0.5;
real y_c = y_max*0.5;
real v_radius = min(x_c,y_c)*0.9;


string[] sizes = {"big", "med", "small"};

struct Polygon {
  string name;
  int sides; 
  void operator init(string name, int sides) {
    this.name = name;
    this.sides = sides;
  }
}


Polygon[] polygons = { Polygon("triangle", 3),
                       Polygon("square", 4),
                       Polygon("pentagon", 5),
                       Polygon("hexagon", 6)
                    };


                    
for (int j=0; j<polygons.length; ++j) {
    string name  = polygons[j].name;
    int    sides = polygons[j].sides;

    for (int i=0; i<=2; ++i) {
        fill(shift(x_c, y_c)*scale(v_radius*(1-0.25*i))*polygon(sides), red);
        xaxis(0, x_max, invisible);
        yaxis(0, y_max, invisible);
        // Clip the elements in the current picture. Prevents the page from being enlarged and needs to be called after all drawing commands.
        clip(box((0,0), (x_max,y_max)));

        // Set the page size. Prevents the page from being shrunken.
        fixedscaling((0,0), (x_max,y_max));
        
        shipout(name+"_"+sizes[i]+".pdf", currentpicture.fit());
        erase();
    }
}


  
  

