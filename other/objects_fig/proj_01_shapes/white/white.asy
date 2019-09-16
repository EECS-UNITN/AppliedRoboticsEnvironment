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

real w = 80;
real h = 160;

fill((x_c-w*0.5,y_c-h*0.5)--(x_c+w*0.5,y_c-h*0.5)--(x_c,y_c+h*0.5)--cycle, cyan+1cm+squarecap+miterjoin);
xaxis(0, x_max, invisible);
yaxis(0, y_max, invisible);
  
// Clip the elements in the current picture. Prevents the page from being enlarged and needs to be called after all drawing commands.
clip(box((0,0), (x_max,y_max)));

// Set the page size. Prevents the page from being shrunken.
fixedscaling((0,0), (x_max,y_max));
  

