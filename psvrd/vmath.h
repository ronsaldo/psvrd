#ifndef PSVRD_VMATH_H
#define PSVRD_VMATH_H

#include <psvrd.h>
#include <tgmath.h>

#define PSVRD_EPSILON ((psvrd_scalar_t)0.00001)

/**
 * Scalar
 */
static inline int psvrd_scalar_closeTo(psvrd_scalar_t a, psvrd_scalar_t b)
{
    psvrd_scalar_t d = a - b;
    return -PSVRD_EPSILON <= d && d <= PSVRD_EPSILON;
}

/**
 * Vector3
 */
static inline psvrd_vector3_t psvrd_vector3_new(psvrd_scalar_t x, psvrd_scalar_t y, psvrd_scalar_t z)
{
    psvrd_vector3_t res = {};
    res.x = x;
    res.y = y;
    res.z = z;

    return res;
}

static inline psvrd_vector3_t psvrd_vector3_add(psvrd_vector3_t a, psvrd_vector3_t b)
{
    return psvrd_vector3_new(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline psvrd_vector3_t psvrd_vector3_sub(psvrd_vector3_t a, psvrd_vector3_t b)
{
    return psvrd_vector3_new(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline psvrd_vector3_t psvrd_vector3_muls(psvrd_vector3_t v, psvrd_scalar_t s)
{
    return psvrd_vector3_new(v.x*s, v.y*s, v.z*s);
}

static inline psvrd_vector3_t psvrd_vector3_divs(psvrd_vector3_t v, psvrd_scalar_t s)
{
    return psvrd_vector3_new(v.x/s, v.y/s, v.z/s);
}

static inline psvrd_scalar_t psvrd_vector3_dot(psvrd_vector3_t a, psvrd_vector3_t b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline psvrd_scalar_t psvrd_vector3_length2(psvrd_vector3_t v)
{
    return psvrd_vector3_dot(v, v);
}

static inline psvrd_scalar_t psvrd_vector3_length(psvrd_vector3_t v)
{
    return sqrt(psvrd_vector3_length2(v));
}

static inline psvrd_vector3_t psvrd_vector3_normalizeNonZero(psvrd_vector3_t v)
{
    psvrd_scalar_t l = psvrd_vector3_length(v);
    if(psvrd_scalar_closeTo(l, 0))
        return v;
    else
        return psvrd_vector3_divs(v, l);
}

static inline psvrd_vector3_t psvrd_vector3_cross(psvrd_vector3_t a, psvrd_vector3_t b)
{
    return psvrd_vector3_new(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x);
}

static inline psvrd_vector3_t psvrd_vector3_orthogonalOfNormalized(psvrd_vector3_t v)
{
    /* https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another */
    return psvrd_vector3_normalizeNonZero(psvrd_vector3_cross(v, psvrd_vector3_new(1.0 - fabs(v.x), 1.0 - fabs(v.y), 1.0 - fabs(v.z))));
}

/**
 * Quaternion
 */
static inline psvrd_quaternion_t psvrd_quaternion_new(psvrd_scalar_t x, psvrd_scalar_t y, psvrd_scalar_t z, psvrd_scalar_t w)
{
    psvrd_quaternion_t res;
    res.x = x;
    res.y = y;
    res.z = z;
    res.w = w;

    return res;
}

static inline psvrd_quaternion_t psvrd_quaternion_newv(psvrd_scalar_t w, psvrd_vector3_t v)
{
    return psvrd_quaternion_new(v.x, v.y, v.z, w);
}

static inline psvrd_quaternion_t psvrd_quaternion_unit(void)
{
    return psvrd_quaternion_new(0, 0, 0, 1);
}

static inline psvrd_quaternion_t psvrd_quaternion_add(psvrd_quaternion_t a, psvrd_quaternion_t b)
{
    return psvrd_quaternion_new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

static inline psvrd_quaternion_t psvrd_quaternion_sub(psvrd_quaternion_t a, psvrd_quaternion_t b)
{
    return psvrd_quaternion_new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

static inline psvrd_quaternion_t psvrd_quaternion_mul(psvrd_quaternion_t a, psvrd_quaternion_t b)
{
    return psvrd_quaternion_new(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}

static inline psvrd_quaternion_t psvrd_quaternion_muls(psvrd_quaternion_t v, psvrd_scalar_t s)
{
    return psvrd_quaternion_new(v.x*s, v.y*s, v.z*s, v.w*s);
}

static inline psvrd_quaternion_t psvrd_quaternion_divs(psvrd_quaternion_t v, psvrd_scalar_t s)
{
    return psvrd_quaternion_new(v.x/s, v.y/s, v.z/s, v.w/s);
}

static inline psvrd_scalar_t psvrd_quaternion_dot(psvrd_quaternion_t a, psvrd_quaternion_t b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

static inline psvrd_scalar_t psvrd_quaternion_length2(psvrd_quaternion_t q)
{
    return psvrd_quaternion_dot(q, q);
}

static inline psvrd_scalar_t psvrd_quaternion_length(psvrd_quaternion_t q)
{
    return sqrt(psvrd_quaternion_length2(q));
}

static inline psvrd_quaternion_t psvrd_quaternion_normalizeNonZero(psvrd_quaternion_t q)
{
    psvrd_scalar_t l = psvrd_quaternion_length(q);
    if(psvrd_scalar_closeTo(l, 0))
        return q;
    else
        return psvrd_quaternion_divs(q, l);
}

static inline psvrd_quaternion_t psvrd_quaternion_conjugate(psvrd_quaternion_t q)
{
    return psvrd_quaternion_new(-q.x, -q.y, -q.z, q.w);
}

static inline psvrd_quaternion_t psvrd_quaternion_negate(psvrd_quaternion_t q)
{
    return psvrd_quaternion_new(-q.x, -q.y, -q.z, -q.w);
}

static inline psvrd_quaternion_t psvrd_quaternion_inverse(psvrd_quaternion_t q)
{
    return psvrd_quaternion_divs(psvrd_quaternion_conjugate(q), psvrd_quaternion_length2(q));
}

static inline psvrd_quaternion_t psvrd_quaternion_toRotateVectorIntoAnotherVector(psvrd_vector3_t a, psvrd_vector3_t b)
{
    /* See https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another*/
    /* Vectors are normalized in this case. */

    psvrd_scalar_t cosTheta = psvrd_vector3_dot(a, b);
    psvrd_scalar_t k = 1.0; /*sqrt(||a||^2 ||b||^2)*/

    /* 180 degrees rotation*/
    if(psvrd_scalar_closeTo(cosTheta, -1))
    {
        //printf("180 degrees\n");
        return psvrd_quaternion_newv(0, psvrd_vector3_orthogonalOfNormalized(a));
    }

    return psvrd_quaternion_normalizeNonZero(psvrd_quaternion_newv(cosTheta + k, psvrd_vector3_cross(a, b)));
}

#endif /* PSVRD_VMATH_H */
