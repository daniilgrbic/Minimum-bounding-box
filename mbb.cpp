#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>

#define CONVHULL_3D_ENABLE
#include "convhull_3d.h"

using namespace std;

typedef long double ftype;
const ftype EPS = 1e-9;

ftype difference_of_products(ftype a, ftype b, ftype c, ftype d) {
    ftype cd = c * d;
    ftype err = std::fma(-c, d, cd);
    ftype dop = std::fma(a, b, -cd);
    return dop + err;
}

vector<vector<ftype>> matrix3x3mul(vector<vector<ftype>> &matrix1, vector<vector<ftype>> &matrix2) {
    vector<vector<ftype>> result {
        {0, 0 ,0}, 
        {0, 0, 0},
        {0, 0, 0}
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int u = 0; u < 3; u++) {
                result[i][j] += matrix1[i][u] * matrix2[u][j];
            }
        }
    }
    return result;
} 

vector<vector<ftype>> matrix3x3inverse(vector<vector<ftype>> &matrix) {
    vector<vector<ftype>> result {
        {0, 0 ,0}, 
        {0, 0, 0},
        {0, 0, 0}
    };
    ftype determinant = matrix[0][0] * difference_of_products(matrix[1][1], matrix[2][2], matrix[1][2], matrix[2][1])
                        - matrix[1][0] * difference_of_products(matrix[0][1], matrix[2][2], matrix[0][2], matrix[2][1])
                        + matrix[2][0] * difference_of_products(matrix[0][1], matrix[1][2], matrix[0][2], matrix[1][1]);
    result[0][0] = difference_of_products(matrix[1][1], matrix[2][2], matrix[1][2], matrix[2][1]) / determinant;
    result[1][0] = difference_of_products(matrix[1][2], matrix[2][0], matrix[1][0], matrix[2][2]) / determinant;
    result[2][0] = difference_of_products(matrix[1][0], matrix[2][1], matrix[1][1], matrix[2][0]) / determinant;

    result[0][1] = difference_of_products(matrix[0][2], matrix[2][1], matrix[0][1], matrix[2][2]) / determinant;
    result[1][1] = difference_of_products(matrix[0][0], matrix[2][2], matrix[0][2], matrix[2][0]) / determinant;
    result[2][1] = difference_of_products(matrix[0][1], matrix[2][0], matrix[0][0], matrix[2][1]) / determinant;

    result[0][2] = difference_of_products(matrix[0][1], matrix[1][2], matrix[0][2], matrix[1][1]) / determinant;
    result[1][2] = difference_of_products(matrix[0][2], matrix[1][0], matrix[0][0], matrix[1][2]) / determinant;
    result[2][2] = difference_of_products(matrix[0][0], matrix[1][1], matrix[0][1], matrix[1][0]) / determinant;
    return result;
} 

vector<ftype> matrix3x3mulvector(vector<vector<ftype>> &matrix, vector<ftype> &v) {
    vector<ftype> result {0, 0, 0};
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i] += (matrix[i][j] * v[j]);
        }
    }
    return result;
}

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

pt3 pt3::operator-(const pt3 &o) const {
    return pt3(x - o.x, y - o.y, z - o.z);
}
pt3 pt3::cross(const pt3 &o) const {
    return pt3(difference_of_products(y, o.z , z, o.y), difference_of_products(z, o.x, x, o.z), difference_of_products(x, o.y, y, o.x));
}
ftype pt3::dot(const pt3 &o) const {
    return x * o.x + y * o.y + z * o.z;
}
void pt3::show() {
    cout << x << " " << y << " " << z << " " << endl;
}
ftype pt3::dist(const pt3 &o) const {
    return sqrt((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y));
}
ftype pt3::dist3d(const pt3 &o) const {
    return sqrt((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y) + (z - o.z) * (z - o.z));
}
ftype pt3::angle(const pt3 &o) const {
    double dot = x * o.x + y * o.y;
    double det = x * o.y - y * o.x;
    double t = atan2(det, dot);
    if(t == -0) {
        t = 0;
    }
    return t;
}
pt3 pt3::rotate(ftype angle) const {
    ftype coss = cos(angle), sinn = sin(angle);
    return pt3(difference_of_products(coss, x, sinn, y), sinn * x + coss * y, 0);
}


void normalize(pt3 &point) {
    ftype np = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if(np > EPS) {
        point.x /= np;
        point.y /= np;
        point.z /= np;
    }
}

ftype ftype_abs(ftype x) {
    return (x >= 0) ? x : -x;
}

bool cmp(const pt3 &point1, const pt3 &point2) { 
    return (point1.x < point2.x) || (ftype_abs(point1.x - point2.x) < EPS && point1.y < point2.y); 
}

bool right_turn(const pt3 &point1, const pt3 &point2, const pt3 &point3) {
    return difference_of_products((point3.x - point1.x), (point2.y - point1.y), (point3.y - point1.y), (point2.x - point1.x)) > 0;
}

pt3 line_inter2d(pt3 A, pt3 B, pt3 C, pt3 D)
{
    // Line AB represented as a1x + b1y = c1
    ftype a1 = B.y - A.y;
    ftype b1 = A.x - B.x;
    ftype c1 = a1 * A.x + b1 * A.y;
  
    // Line CD represented as a2x + b2y = c2
    ftype a2 = D.y - C.y;
    ftype b2 = C.x - D.x;
    ftype c2 = a2 * C.x+ b2 * C.y;
  
    ftype determinant = difference_of_products(a1, b2, a2, b1);
  
    pt3 intersection(difference_of_products(b2, c1, b1, c2) / determinant, difference_of_products(a1, c2, a2, c1) / determinant, 0);
    if(intersection.x == -0) {
        intersection.x = 0;
    }
    if(intersection.y == -0) {
        intersection.y = 0;
    }

    return intersection;    
}

void OBB(vector<pt3> &points, pt3 v, ftype &volume, vector<pt3> &lowerbase, vector<pt3> &upperbase) {
    bool indx = ftype_abs(v.x) < EPS, indy = ftype_abs(v.y) < EPS, indz = ftype_abs(v.z) < EPS;
    if(indx && indy && indz) {
        return ;
    }

    vector<pt3> tmp;
    for(auto u : points) {
        tmp.push_back(u);
    }

// Make base transformation matrix
    vector<vector<ftype>> trmatrix {
        {1, 0 ,0}, 
        {0, 1, 0},
        {0, 0, 1}
    };
    vector<vector<ftype>> trmatrixinverse (trmatrix.begin(), trmatrix.end());
    if(indx && indy) {
    }
    else {
        pt3 tz = v;
        pt3 tx, ty;
        pt3 xx(1, 0, 0), yy(0, 1, 0);
        if(!(indy && indz)) {
            tx = v.cross(xx);
            normalize(tx);
            ty = tz.cross(tx);
            normalize(ty);
        }
        else {
            tx = v.cross(yy);
            normalize(tx);
            ty = tz.cross(tx);
            normalize(ty);
        }

        vector<vector<ftype>> C {
            {tx.x, ty.x, tz.x},
            {tx.y, ty.y, tz.y},
            {tx.z, ty.z, tz.z}
        };
        trmatrixinverse = C;
        trmatrix = matrix3x3inverse(C);
    }

// Transform point coordinates
    int n = tmp.size();
    for(int i = 0; i < n; i++) {
        vector <ftype> tt {tmp[i].x, tmp[i].y, tmp[i].z}; 
        tt = matrix3x3mulvector(trmatrix, tt); 
        tmp[i].x = tt[0]; tmp[i].y = tt[1]; tmp[i].z = tt[2]; 
    }
    sort(tmp.begin(), tmp.end(), cmp);

// Find 2D convex hull
    vector<pt3> ch2d, ch2dupper, ch2dlower;
    ch2dupper.push_back(tmp[0]);
    ch2dupper.push_back(tmp[1]);
    ch2dlower.push_back(tmp[n - 1]);
    ch2dlower.push_back(tmp[n - 2]);
    for(int i = 2; i < n; i++) {
        while(ch2dupper.size() > 1 && (!right_turn(ch2dupper[ch2dupper.size() - 2], ch2dupper[ch2dupper.size() - 1], tmp[i]))) {
            ch2dupper.pop_back();
        }
        ch2dupper.push_back(tmp[i]);
    }
    for(int i=2; i< n; i++) {
        while(ch2dlower.size() > 1 && (!right_turn(ch2dlower[ch2dlower.size() - 2], ch2dlower[ch2dlower.size()-1], tmp[n - i - 1]))) {
            ch2dlower.pop_back();
        }
        ch2dlower.push_back(tmp[n - i - 1]);
    }
    for(int i = 0; i < ch2dupper.size(); i++) {
        ch2d.push_back(ch2dupper[i]);
    }
    for(int i = 1; i < ch2dlower.size() - 1; i++) {
        ch2d.push_back(ch2dlower[i]);
    }

    ftype zmin = tmp[0].z, zmax =  tmp[0].z;
    for(auto u : tmp) {
        if(zmin > u.z) {
            zmin = u.z;
        }
        if(zmax < u.z) {
            zmax = u.z;
        }
    }

// Set calipers to initial positions 
    pt3 caliper[4] = {pt3(1, 0, 0), pt3(0, -1, 0), pt3(-1, 0, 0), pt3(0, 1, 0)};
    int index[4];
    ftype xmin = 1000000, xmax = -1000000, ymin = 1000000, ymax = -1000000;

    for(int i = 0; i < ch2d.size(); i++) {
        if(ch2d[i].x < xmin) {
            xmin = ch2d[i].x;
            index[3] = i;
        }
        if(ch2d[i].x > xmax) {
            xmax = ch2d[i].x;
            index[1] = i;
        }
        if(ch2d[i].y > ymax) {
            ymax = ch2d[i].y;
            index[0] = i;
        }
        if(ch2d[i].y < ymin) {
            ymin = ch2d[i].y;
            index[2] = i;
        }
    }

// Calculate initial caliper intersections and area
    pt3 pt[4];
    for(int i = 0; i < 4; i++) {
        pt3 t(ch2d[index[i]].x + caliper[i].x, ch2d[index[i]].y + caliper[i].y, 0);
        int i1 = (i + 1) % 4;
        pt3 g(ch2d[index[i1]].x + caliper[i1].x, ch2d[index[i1]].y + caliper[i1].y, 0);
        pt[i] = line_inter2d(ch2d[index[i]], t, ch2d[index[i1]], g);
    }
    ftype area = pt[0].dist(pt[1]) * pt[1].dist(pt[2]);

// Rotate calipers
    for(int q = 0; q < ch2d.size(); q++) {
        ftype minangle = 50;
        int ind;
        for(int i = 0; i < 4; i++) {
            int k = (index[i] + 1) % ch2d.size();
            pt3 p1(ch2d[k].x - ch2d[index[i]].x, ch2d[k].y - ch2d[index[i]].y, 0);
            ftype tangle = p1.angle(caliper[i]);
            if(tangle < minangle) {
                minangle = tangle; 
                ind = i;
            }
        }
        for(int i = 0; i < 4; i++) {
            caliper[i] = caliper[i].rotate((-1) * minangle);
        }
        index[ind]++;
        index[ind] %= ch2d.size();

        pt3 ptt[4];
        for(int i = 0; i < 4; i++) {
            pt3 t(ch2d[index[i]].x + caliper[i].x, ch2d[index[i]].y + caliper[i].y, 0);
            int i1 = (i + 1) % 4;
            pt3 g(ch2d[index[i1]].x + caliper[i1].x, ch2d[index[i1]].y + caliper[i1].y, 0);
            ptt[i] = line_inter2d(ch2d[index[i]], t, ch2d[index[i1]], g);
        }
        ftype tarea = ptt[0].dist(ptt[1]) * ptt[1].dist(ptt[2]);
        if(tarea < area) {
            area = tarea;
            for(int i = 0; i < 4; i++) {
                pt[i] = ptt[i];
            }
        }
    }

// Check if OBB volume is smaller than current
    ftype tvolume = (zmax - zmin) * area;
    if(tvolume < volume) {
        volume = tvolume;
        vector<ftype> pv(3);
        for(int i = 0; i < 4; i++) {
            pv[0] = pt[i].x;
            pv[1] = pt[i].y;
            pv[2] = zmin;
            pv = matrix3x3mulvector(trmatrixinverse, pv);
            lowerbase[i].x = pv[0];
            lowerbase[i].y = pv[1];
            lowerbase[i].z = pv[2];
        }
        for(int i = 0; i < 4; i++) {
            pv[0] = pt[i].x;
            pv[1] = pt[i].y;
            pv[2] = zmax;
            pv = matrix3x3mulvector(trmatrixinverse, pv);
            upperbase[i].x = pv[0];
            upperbase[i].y = pv[1];
            upperbase[i].z = pv[2];
        }
    }
}

void MBBapproximation(vector<pt3> &points, vector<pt3> &lowerbase, vector<pt3> &upperbase) {
// Find convexhull
    int nVertices = points.size();
    ch_vertex* vertices;
    vertices = (ch_vertex*)malloc(nVertices * sizeof(ch_vertex));
    for (int i = 0; i < nVertices; i++) {
        vertices[i].x = points[i].x;
        vertices[i].y = points[i].y;
        vertices[i].z = points[i].z;
    }

    int* faceIndices = NULL;
    int nFaces;
    convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);
    //convhull_3d_export_obj(vertices, nVertices, faceIndices, nFaces, 1, "hull");

    unordered_set<int> chindexes;
    for(int i = 0; i < 3*nFaces; i++) {
        chindexes.insert(faceIndices[i]);
    }
    vector<pt3> convexhull;
    for(auto u : chindexes) {
        convexhull.push_back(points[u]);
    }

    free(vertices);
    free(faceIndices);

// Find two most distant points and initialize obb orientation
    int index1, index2;
    ftype maxdist = 0;
    for(int i = 0; i < convexhull.size(); i++) {
        for(int j = i + 1; j < convexhull.size(); j++) {
            ftype tmpdist = convexhull[i].dist3d(convexhull[j]);
            if(tmpdist > maxdist) {
                index1 = i;
                index2 = j;
                maxdist = tmpdist;
            }
        }
    }

    pt3 vinit = convexhull[index1] - convexhull[index2];
    //pt3 vinit(1, 5, 3.1);
    ftype volume = 100000000;
    normalize(vinit);

    /*OBB(convexhull, vinit, volume, lowerbase, upperbase);
    cout << "Volumen " << volume << endl;
    return;*/

// Iterate through selected orientations
    int d = 10;
    for(int i1 = 0; i1 < d; i1++) {
        for(int i2 = 0; i2 < d; i2++) {
            for(int i3 = 0; i3 < d; i3++) {
                pt3 v(vinit.x * i1, vinit.y * i2, vinit.z * i3);
                normalize(v);
                if(v.x == -0) {
                    v.x = 0;
                }
                if(v.y == -0) {
                    v.y = 0;
                }
                if(v.z == -0) {
                    v.z = 0;
                }
                OBB(convexhull, v, volume, lowerbase, upperbase);
            }
        }
    }
    cout << "Volumen " << volume << endl;
}

int test() {

    vector<pt3> points;
    vector<pt3> lowerbase(4), upperbase(4);
    MBBapproximation(points, lowerbase, upperbase);

    return 0;
}