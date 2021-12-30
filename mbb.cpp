#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <numeric>

#define CONVHULL_3D_ENABLE
#include "convhull_3d.h"

#include "mbb.h"

using namespace std;

typedef long double ftype;
const ftype EPS = 1e-9;
const ftype MAX = 1000000000;

ftype differenceOfProducts(ftype a, ftype b, ftype c, ftype d) {
    ftype cd = c * d;
    ftype err = std::fma(-c, d, cd);
    ftype dop = std::fma(a, b, -cd);
    return dop + err;
}

vector<vector<ftype>> matrix3x3Inverse(vector<vector<ftype>> &matrix) {
    vector<vector<ftype>> result {
            {0, 0 ,0},
            {0, 0, 0},
            {0, 0, 0}
    };
    ftype determinant = matrix[0][0] * differenceOfProducts(matrix[1][1], matrix[2][2], matrix[1][2], matrix[2][1])
                        - matrix[1][0] * differenceOfProducts(matrix[0][1], matrix[2][2], matrix[0][2], matrix[2][1])
                        + matrix[2][0] * differenceOfProducts(matrix[0][1], matrix[1][2], matrix[0][2], matrix[1][1]);
    result[0][0] = differenceOfProducts(matrix[1][1], matrix[2][2], matrix[1][2], matrix[2][1]) / determinant;
    result[1][0] = differenceOfProducts(matrix[1][2], matrix[2][0], matrix[1][0], matrix[2][2]) / determinant;
    result[2][0] = differenceOfProducts(matrix[1][0], matrix[2][1], matrix[1][1], matrix[2][0]) / determinant;

    result[0][1] = differenceOfProducts(matrix[0][2], matrix[2][1], matrix[0][1], matrix[2][2]) / determinant;
    result[1][1] = differenceOfProducts(matrix[0][0], matrix[2][2], matrix[0][2], matrix[2][0]) / determinant;
    result[2][1] = differenceOfProducts(matrix[0][1], matrix[2][0], matrix[0][0], matrix[2][1]) / determinant;

    result[0][2] = differenceOfProducts(matrix[0][1], matrix[1][2], matrix[0][2], matrix[1][1]) / determinant;
    result[1][2] = differenceOfProducts(matrix[0][2], matrix[1][0], matrix[0][0], matrix[1][2]) / determinant;
    result[2][2] = differenceOfProducts(matrix[0][0], matrix[1][1], matrix[0][1], matrix[1][0]) / determinant;
    return result;
}

vector<ftype> matrix3x3MulVector(vector<vector<ftype>> &matrix, vector<ftype> &v) {
    vector<ftype> result {0, 0, 0};
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i] += (matrix[i][j] * v[j]);
        }
    }
    return result;
}

pt3 pt3::operator-(const pt3 &o) const {
    return pt3(x - o.x, y - o.y, z - o.z);
}
pt3 pt3::cross(const pt3 &o) const {
    return pt3(differenceOfProducts(y, o.z , z, o.y), differenceOfProducts(z, o.x, x, o.z), differenceOfProducts(x, o.y, y, o.x));
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
ftype pt3::dist3D(const pt3 &o) const {
    return sqrt((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y) + (z - o.z) * (z - o.z));
}
ftype pt3::angle(const pt3 &o) const {
    double dot = x * o.x + y * o.y;
    double det = x * o.y - y * o.x;
    double t = atan2(det, dot);
    
    return t;
}
pt3 pt3::rotate(ftype angle) const {
    ftype coss = cos(angle), sinn = sin(angle);
    return pt3(differenceOfProducts(coss, x, sinn, y), sinn * x + coss * y, 0);
}


void normalize(pt3 &point) {
    ftype norm = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if(norm > EPS) {
        point.x /= norm;
        point.y /= norm;
        point.z /= norm;
    }
}

ftype ftype_abs(ftype x) {
    return (x >= 0) ? x : -x;
}

bool cmp(const pt3 &point1, const pt3 &point2) {
    return (point1.x < point2.x) || (ftype_abs(point1.x - point2.x) < EPS && point1.y < point2.y);
}

bool rightTurn(const pt3 &point1, const pt3 &point2, const pt3 &point3) {
    return differenceOfProducts((point3.x - point1.x), (point2.y - point1.y), (point3.y - point1.y), (point2.x - point1.x)) > 0;
}

pt3 lineInter2D(pt3 A, pt3 B, pt3 C, pt3 D) {
// Line AB represented as a1x + b1y = c1
    ftype a1 = B.y - A.y;
    ftype b1 = A.x - B.x;
    ftype c1 = a1 * A.x + b1 * A.y;

// Line CD represented as a2x + b2y = c2
    ftype a2 = D.y - C.y;
    ftype b2 = C.x - D.x;
    ftype c2 = a2 * C.x+ b2 * C.y;

    ftype determinant = differenceOfProducts(a1, b2, a2, b1);

    return pt3(differenceOfProducts(b2, c1, b1, c2) / determinant, differenceOfProducts(a1, c2, a2, c1) / determinant, 0);
}

void orientedBoundingBox(vector<pt3> points, pt3 orientation, ftype &volume, vector<pt3> &lowerBase, vector<pt3> &upperBase) {
    bool indX = ftype_abs(orientation.x) < EPS, indY = ftype_abs(orientation.y) < EPS, indZ = ftype_abs(orientation.z) < EPS;
    if(indX && indY && indZ) {
        return ;
    }

// Make base transformation matrix
    vector<vector<ftype>> trMatrix {
            {1, 0 ,0},
            {0, 1, 0},
            {0, 0, 1}
    };
    vector<vector<ftype>> trMatrixInverse (trMatrix.begin(), trMatrix.end());
    if(indX && indY) {
    }
    else {
        pt3 baseZ = orientation;
        pt3 baseX, baseY;
        pt3 tempX(1, 0, 0), tempY(0, 1, 0);
        if(!(indY && indZ)) {
            baseX = orientation.cross(tempX);
            normalize(baseX);
            baseY = baseZ.cross(baseX);
            normalize(baseY);
        }
        else {
            baseX = orientation.cross(tempY);
            normalize(baseX);
            baseY = baseZ.cross(baseX);
            normalize(baseY);
        }

        vector<vector<ftype>> C {
                {baseX.x, baseY.x, baseZ.x},
                {baseX.y, baseY.y, baseZ.y},
                {baseX.z, baseY.z, baseZ.z}
        };
        trMatrixInverse = C;
        trMatrix = matrix3x3Inverse(C);
    }

// Transform point coordinates
    int nPoints = points.size();
    ftype zMin = MAX, zMax = -MAX;
    for(int i = 0; i < nPoints; i++) {
        vector <ftype> tempVector {points[i].x, points[i].y, points[i].z};
        tempVector = matrix3x3MulVector(trMatrix, tempVector);
        points[i].x = tempVector[0]; points[i].y = tempVector[1]; points[i].z = tempVector[2];
        if(zMin > points[i].z) {
            zMin = points[i].z;
        }
        if(zMax < points[i].z) {
            zMax = points[i].z;
        }
    }
    sort(points.begin(), points.end(), cmp);

    vector<pt3> tempPoints;
    tempPoints.push_back(points[0]);
    for(int i = 1; i < nPoints; i++) {
        if(ftype_abs(tempPoints[tempPoints.size() - 1].x - points[i].x) < EPS && ftype_abs(tempPoints[tempPoints.size() - 1].y - points[i].y) < EPS) {
            continue;
        }
        tempPoints.push_back(points[i]);
    }
    nPoints = tempPoints.size();

// Find 2D convex hull and set calipers to initial positions
    vector<pt3> CH2D, upperCH2D, lowerCH2D;
    upperCH2D.push_back(tempPoints[0]);
    upperCH2D.push_back(tempPoints[1]);
    lowerCH2D.push_back(tempPoints[nPoints - 1]);
    lowerCH2D.push_back(tempPoints[nPoints - 2]);
    for(int i = 2; i < nPoints; i++) {
        while(upperCH2D.size() > 1 && (!rightTurn(upperCH2D[upperCH2D.size() - 2], upperCH2D[upperCH2D.size() - 1], tempPoints[i]))) {
            upperCH2D.pop_back();
        }
        upperCH2D.push_back(tempPoints[i]);
        
    }
    for(int i = 2; i < nPoints; i++) {
        while(lowerCH2D.size() > 1 && (!rightTurn(lowerCH2D[lowerCH2D.size() - 2], lowerCH2D[lowerCH2D.size() - 1], tempPoints[nPoints - i - 1]))) {
            lowerCH2D.pop_back();
        }
        lowerCH2D.push_back(tempPoints[nPoints - i - 1]);
    }

    pt3 caliper[4] = {pt3(1, 0, 0), pt3(0, -1, 0), pt3(-1, 0, 0), pt3(0, 1, 0)};
    int caliperIndex[4] = {0, 0, 0, 0};
    ftype xMin = MAX, xMax = -MAX, yMin = MAX, yMax = -MAX;
    for(int i = 0; i < upperCH2D.size(); i++) {
        CH2D.push_back(upperCH2D[i]);
        if(CH2D[i].x < xMin) {
            xMin = CH2D[i].x;
            caliperIndex[3] = i;
        }
        if(CH2D[i].x > xMax) {
            xMax = CH2D[i].x;
            caliperIndex[1] = i;
        }
        if(CH2D[i].y > yMax) {
            yMax = CH2D[i].y;
            caliperIndex[0] = i;
        }
        if(CH2D[i].y < yMin) {
            yMin = CH2D[i].y;
            caliperIndex[2] = i;
        }
    }
    int tempInd;
    for(int i = 1; i < lowerCH2D.size() - 1; i++) {
        CH2D.push_back(lowerCH2D[i]);
        tempInd = CH2D.size() - 1;
        if(CH2D[tempInd].x < xMin) {
            xMin = CH2D[tempInd].x;
            caliperIndex[3] = tempInd;
        }
        if(CH2D[tempInd].x > xMax) {
            xMax = CH2D[tempInd].x;
            caliperIndex[1] = tempInd;
        }
        if(CH2D[tempInd].y > yMax) {
            yMax = CH2D[tempInd].y;
            caliperIndex[0] = tempInd;
        }
        if(CH2D[tempInd].y < yMin) {
            yMin = CH2D[tempInd].y;
            caliperIndex[2] = tempInd;
        }
    }

// Calculate initial caliper intersections and area
    pt3 rectanglePoints[4];
    for(int i = 0; i < 4; i++) {
        pt3 tempPoint1(CH2D[caliperIndex[i]].x + caliper[i].x, CH2D[caliperIndex[i]].y + caliper[i].y, 0);
        int i1 = (i + 1) % 4;
        pt3 tempPoint2(CH2D[caliperIndex[i1]].x + caliper[i1].x, CH2D[caliperIndex[i1]].y + caliper[i1].y, 0);
        rectanglePoints[i] = lineInter2D(CH2D[caliperIndex[i]], tempPoint1, CH2D[caliperIndex[i1]], tempPoint2);
    }
    ftype area = rectanglePoints[0].dist(rectanglePoints[1]) * rectanglePoints[1].dist(rectanglePoints[2]);

// Rotate calipers
    for(int q = 0; q < CH2D.size(); q++) {
        ftype minAngle = 50;
        int ind;
        for(int i = 0; i < 4; i++) {
            int k = (caliperIndex[i] + 1) % CH2D.size();
            pt3 tempPoint(CH2D[k].x - CH2D[caliperIndex[i]].x, CH2D[k].y - CH2D[caliperIndex[i]].y, 0);
            ftype tempAngle = tempPoint.angle(caliper[i]);
            if(tempAngle < minAngle) {
                minAngle = tempAngle;
                ind = i;
            }
        }
        for(int i = 0; i < 4; i++) {
            caliper[i] = caliper[i].rotate((-1) * minAngle);
        }
        caliperIndex[ind] = (caliperIndex[ind] + 1) % CH2D.size();

        pt3 tempRectanglePoints[4];
        for(int i = 0; i < 4; i++) {
            pt3 tempPoint1(CH2D[caliperIndex[i]].x + caliper[i].x, CH2D[caliperIndex[i]].y + caliper[i].y, 0);
            int i1 = (i + 1) % 4;
            pt3 tempPoint2(CH2D[caliperIndex[i1]].x + caliper[i1].x, CH2D[caliperIndex[i1]].y + caliper[i1].y, 0);
            tempRectanglePoints[i] = lineInter2D(CH2D[caliperIndex[i]], tempPoint1, CH2D[caliperIndex[i1]], tempPoint2);
        }
        ftype tempArea = tempRectanglePoints[0].dist(tempRectanglePoints[1]) * tempRectanglePoints[1].dist(tempRectanglePoints[2]);
        if(tempArea < area) {
            area = tempArea;
            for(int i = 0; i < 4; i++) {
                rectanglePoints[i] = tempRectanglePoints[i];
            }
        }
    }

// Check if OBB volume is smaller than current
    ftype tempVolume = (zMax - zMin) * area;
    if(tempVolume < volume) {
        volume = tempVolume;
        vector<ftype> tempBase(3);
        for(int i = 0; i < 4; i++) {
            tempBase[0] = rectanglePoints[i].x;
            tempBase[1] = rectanglePoints[i].y;
            tempBase[2] = zMin;
            tempBase = matrix3x3MulVector(trMatrixInverse, tempBase);
            lowerBase[i].x = tempBase[0];
            lowerBase[i].y = tempBase[1];
            lowerBase[i].z = tempBase[2];
        }
        for(int i = 0; i < 4; i++) {
            tempBase[0] = rectanglePoints[i].x;
            tempBase[1] = rectanglePoints[i].y;
            tempBase[2] = zMax;
            tempBase = matrix3x3MulVector(trMatrixInverse, tempBase);
            upperBase[i].x = tempBase[0];
            upperBase[i].y = tempBase[1];
            upperBase[i].z = tempBase[2];
        }
    }
}

void mbbApproximation(vector<pt3> &points, vector<pt3> &lowerBase, vector<pt3> &upperBase) {
// Find convexhull
    int nVertices = points.size();
    ch_vertex* vertices;
    vertices = (ch_vertex*)malloc(nVertices * sizeof(ch_vertex));
    for (int i = 0; i < nVertices; i++) {
        vertices[i].x = points[i].x;
        vertices[i].y = points[i].y;
        vertices[i].z = points[i].z;
    }
    int* faceIndices = nullptr;
    int nFaces;
    convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);

    set<int> chIndexes;
    for(int i = 0; i < 3 * nFaces; i++) {
        chIndexes.insert(faceIndices[i]);
    }
    vector<pt3> convexHull;
    for(auto u : chIndexes) {
        convexHull.push_back(points[u]);
    }

    free(vertices);
    free(faceIndices);

// Find two most distant points and initialize obb orientation
    int index1, index2;
    ftype maxDist = 0;
    for(int i = 0; i < convexHull.size(); i++) {
        for(int j = i + 1; j < convexHull.size(); j++) {
            ftype tempDist = convexHull[i].dist3D(convexHull[j]);
            if(tempDist > maxDist) {
                index1 = i;
                index2 = j;
                maxDist = tempDist;
            }
        }
    }
    pt3 orientation = convexHull[index1] - convexHull[index2];
    ftype volume = MAX;
    normalize(orientation);

// Iterate through selected orientations
    orientedBoundingBox(convexHull, pt3(1, 0, 0), volume, lowerBase, upperBase);
    orientedBoundingBox(convexHull, pt3(0, 1, 0), volume, lowerBase, upperBase);
    orientedBoundingBox(convexHull, pt3(0, 0, 1), volume, lowerBase, upperBase);
    int d = 11;
    set<pair<int, int>> set1;
    for(int i1 = 1; i1 < d; i1++) {
        for(int i2 = 1; i2 < d; i2++) {
            int gcd = std::gcd(i1, i2);
            pair<int, int> temp = {i1 / gcd, i2 / gcd};
            if(set1.count(temp)) {
                continue;
            }
            set1.insert(temp);
            pt3 tempOrientation1(orientation.x * i1, orientation.y * i2, 0);
            normalize(tempOrientation1);
            orientedBoundingBox(convexHull, tempOrientation1, volume, lowerBase, upperBase);
            pt3 tempOrientation2(orientation.x * i1, 0, orientation.z * i2);
            normalize(tempOrientation2);
            orientedBoundingBox(convexHull, tempOrientation2, volume, lowerBase, upperBase);
            pt3 tempOrientation3(0, orientation.y * i1, orientation.z * i2);
            normalize(tempOrientation3);
            orientedBoundingBox(convexHull, tempOrientation3, volume, lowerBase, upperBase);
        }
    }
    set<tuple<int, int, int>> set2;
    for(int i1 = 1; i1 < d; i1++) {
        for(int i2 = 1; i2 < d; i2++) {
            for(int i3 = 1; i3 < d; i3++) {
                int gcd = std::gcd(i1, std::gcd(i2, i3));
                tuple<int, int, int> temp = make_tuple(i1 / gcd, i2 / gcd, i3 / gcd);
                if(set2.count(temp)) {
                    continue;
                }
                set2.insert(temp);
                pt3 tempOrientation(orientation.x * i1, orientation.y * i2, orientation.z * i3);
                normalize(tempOrientation);
                orientedBoundingBox(convexHull, tempOrientation, volume, lowerBase, upperBase);
            }
        }
    }
    cout << "MBB volume: " << volume << endl;
}