unitsize(1mm);

// Requested page boundaries.
real x_max = 189;
real y_max = 189;

import settings;
settings.render = 16;

import graph;

texpreamble("\usepackage{helvet}");
texpreamble("\renewcommand{\familydefault}{\sfdefault}");

real x_c = x_max*0.5;
real y_c = y_max*0.5;
real v_radius = min(x_c,y_c);

pen v_color = green;

for (int i=1; i<=9; ++i) {
  fill(circle((x_c,y_c), v_radius), v_color);
  string txt = "\textbf{"+string(i)+"}";
  label(txt, (x_c,y_c), black+fontsize(6*v_radius));
  xaxis(0, x_max, invisible);
  yaxis(0, y_max, invisible);
  
  //xlimits(0, x_max, Crop);
  //ylimits(0, y_max, Crop);
  
  // Clip the elements in the current picture. Prevents the page from being enlarged and needs to be called after all drawing commands.
  clip(box((0,0), (x_max,y_max)));

  // Set the page size. Prevents the page from being shrunken.
  fixedscaling((0,0), (x_max,y_max));
  
  shipout(string(i)+".png", currentpicture.fit());
  erase();
}

