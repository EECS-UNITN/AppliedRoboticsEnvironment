unitsize(1cm);

import settings;
settings.render = 16;

import graph;

real v_radius = 1;

texpreamble("\usepackage{helvet}");
texpreamble("\renewcommand{\familydefault}{\sfdefault}");

                   
for (int i=0; i<=9; ++i) {
  erase();
  string txt = "\textbf{"+string(i)+"}";
  label(scale(5*v_radius)*txt);
  string name = string(i) + ".pdf";
  shipout(name, scale(0.2)*currentpicture.fit());
}
