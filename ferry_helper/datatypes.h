#ifndef DATA_HELPER_H
#define DATA_HELPER_H

struct point2D {
    double x;
    double y;
};

inline double dist(const point2D& a, const point2D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double distSq(const point2D& a, const point2D& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

struct color {
    int r;
    int g;
    int b;
};

color white = { 255, 255, 255 };
color black = { 0, 0, 0 };
color red = { 255, 0, 0 };
color green = { 0, 255, 0 };
color blue = { 0, 0, 255 };
color yellow = { 255, 255, 0 };
color purple = { 255, 0, 255 };
color orange = { 255, 165, 0 };
color gray = { 128, 128, 128 };
color pink = { 255, 192, 203 };
color brown = { 165, 42, 42 };
color teal = { 0, 128, 128 };
color indigo = { 75, 0, 130 };
color lime = { 0, 255, 0 };
color olive = { 128, 128, 0 };
color navy = { 0, 0, 128 };
color silver = { 192, 192, 192 };
color maroon = { 128, 0, 0 };
color aqua = { 0, 255, 255 };
color fuchsia = { 255, 0, 255 };
color crimson = { 220, 20, 60 };
color gold = { 255, 215, 0 };
color tomato = { 255, 99, 71 };
color violet = { 238, 130, 238 };
color cyan = { 0, 255, 255 };
color magenta = { 255, 0, 255 };
color cornsilk = { 255, 248, 220 };
color ghostwhite = { 248, 248, 255 };
color honeydew = { 240, 255, 240 };

std::vector<color> colors = {
    white, black, red, green, blue, yellow, purple, orange, gray, pink, brown, teal, indigo, lime, olive, navy, silver, maroon, aqua, fuchsia, crimson, gold, tomato, violet, cyan, magenta, cornsilk, ghostwhite, honeydew
};

#endif // DATA_HELPER_H