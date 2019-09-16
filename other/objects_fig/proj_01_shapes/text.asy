unitsize(1cm);

import settings;
settings.render = 16;

import graph;

texpreamble("\usepackage{helvet}");
texpreamble("\renewcommand{\familydefault}{\sfdefault}");

real xmin, xmax, ymin, ymax;
xmin = -1; 
ymin = -1;
xmax =  1;
ymax =  1;

real v_radius = 2;
pen v_color = green;

struct Victim {
  int id;
  pair pos;
  void operator init(int id, pair pos) {
    this.id = id;
    this.pos = pos;
  }
};


Victim[] victims = { Victim(5, (0,0)) };

                   
for (int i=0; i<victims.length; ++i) {
  fill(circle(victims[i].pos, v_radius), v_color);
  string txt = "\textbf{"+string(victims[i].id)+"}";
  label(scale(5*v_radius)*txt, victims[i].pos);
}

xaxis(xmin-1, xmax+1, invisible);
yaxis(ymin-1, ymax+1, invisible);

shipout(scale(0.2)*currentpicture.fit());
