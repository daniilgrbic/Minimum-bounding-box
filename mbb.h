#ifndef MINIMUM_BOUNDING_BOX_MBB_H
#define MINIMUM_BOUNDING_BOX_MBB_H

typedef long double ftype;

struct pt3 {
    ftype x, y, z;
    pt3(ftype x = 0, ftype y = 0, ftype z = 0) : x(x), y(y), z(z) {}
    pt3 operator-(const pt3 &o) const;
    pt3 cross(const pt3 &o) const;
    ftype dot(const pt3 &o) const;
    void show();
    ftype dist(const pt3 &o) const;
    ftype dist3d(const pt3 &o) const;
    ftype angle(const pt3 &o) const;
    pt3 rotate(ftype angle) const;
};

void MBBapproximation(
        std::vector<pt3> &points,
        std::vector<pt3> &lowerbase,
        std::vector<pt3> &upperbase
);

#endif //MINIMUM_BOUNDING_BOX_MBB_H
