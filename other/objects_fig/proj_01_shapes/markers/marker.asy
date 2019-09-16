unitsize(1mm);

// Requested page boundaries.
real x_max = 210;
real y_max = 297;

import settings;
settings.render = 16;

import graph;

texpreamble("\usepackage{helvet}");
texpreamble("\renewcommand{\familydefault}{\sfdefault}");

real x_c = x_max*0.5;
real y_c = y_max*0.5;
real v_radius = 25;

pen v_color = black;

for (int i=1; i<=3; ++i) {
  draw(circle((x_c,y_c/2*i), v_radius), v_color+12bp);
}

xaxis(0, x_max, invisible);
yaxis(0, y_max, invisible);
  
  
// Clip the elements in the current picture. Prevents the page from being enlarged and needs to be called after all drawing commands.
clip(box((0,0), (x_max,y_max)));

// Set the page size. Prevents the page from being shrunken.
fixedscaling((0,0), (x_max,y_max));



