/*
* This file is part of ArmarX.
*
* ArmarX is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* ArmarX is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "Line.h"
#include "Triangle.h"
#include "Primitive.h"
#include "float.h"

using namespace math;



Line::Line(Vec3 pos, Vec3 dir)
    : pos(pos), dir(dir)
{
}

Line Line::Normalized()
{
    return Line(pos, dir.normalized());
}

Vec3 Line::Get(float t)
{
    return pos + t * dir;
}

Vec3 Line::GetDerivative(float)
{
    return dir;
}

Vec3 Line::GetClosestPoint(Vec3 p)
{
    return pos - (pos - p).dot(dir) * dir / dir.squaredNorm();
}

float Line::GetT(Vec3 p)
{
    return (p - pos).dot(dir) / dir.squaredNorm();

}

std::string Line::ToString()
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir << ")";
    return ss.str();
}


//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool Line::IntersectsTriangle(Triangle tri, float &out)
{
    const float EPS = 0.000; //TODO
    Vec3 e1, e2;  //Edge1, Edge2
    Vec3 P, Q, T;
    float det, inv_det, u, v;
    float t;
    Vec3 V1 = tri.P1();
    Vec3 V2 = tri.P2();
    Vec3 V3 = tri.P3();

    //Find vectors for two edges sharing V1
    e1 = V2 -V1;
    e2 = V3 -V1;

    //Begin calculating determinant - also used to calculate u parameter
    P = dir.cross(e2);
    //if determinant is near zero, ray lies in plane of triangle
    det =e1.dot( P);
    //NOT CULLING
    if(det > -EPS && det < EPS) return 0;
    inv_det = 1.f / det;

    //calculate distance from V1 to ray origin
    T = pos - V1;

    //Calculate u parameter and test bound
    u = T.dot(P) * inv_det;
    //The intersection lies outside of the triangle
    if(u < 0.f || u > 1.f) return 0;

    //Prepare to test v parameter
   Q = T.cross(e1);

    //Calculate V parameter and test bound
    v = dir.dot(Q) * inv_det;
    //The intersection lies outside of the triangle
    if(v < 0.f || u + v  > 1.f) return 0;

    t = e2.dot(Q) * inv_det;

    if(t > EPS) { //ray intersection
      out = t;
      return 1;
    }

    // No hit, no win
    return 0;
}

bool Line::IntersectsPrimitive(PrimitivePtr p, float &out)
{
    float min = FLT_MAX;
    float t;
    for(Triangle tri : *p){
        if (IntersectsTriangle(tri, t)){
            if(t<min) min = t;
        }
    }
    out = min;
    return out < FLT_MAX;
}

Line Line::FromPoints(Vec3 p1, Vec3 p2)
{
    return Line(p1, p2 - p1);
}






