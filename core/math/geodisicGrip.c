/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      geodisicGrip.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月14日星期四
 * \brief     地理分区算法
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月14日星期四
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/

/*
 * This comment section explains the basic idea behind the implementation.
 *
 * Vectors difference notation
 * ===========================
 * Let v and w be vectors. For readability purposes, unless explicitly
 * otherwise noted, the notation vw will be used to represent w - v.
 *
 * Relationship between a vector and a triangle in 3d space
 * ========================================================
 * Vector in the area of a triangle
 * --------------------------------
 * Let T = (a, b, c) be a triangle, where a, b and c are also vectors and
 * linearly independent. A vector inside that triangle can be written as one of
 * its vertices plus the sum of the positively scaled vectors from that vertex
 * to the other ones. Taking a as the first vertex, a vector p in the area
 * formed by T can be written as:
 *
 *     p = a + w_ab * ab + w_ac * ac
 *
 * It's fairly easy to see that if p is in the area formed by T, then w_ab >= 0
 * and w_ac >= 0. That vector p can also be written as:
 *
 *     p = b + w_ba * ba + w_bc * bc
 *
 * It's easy to check that the triangle formed by (a + w_ab * ab, b + w_ba *
 * ba, p) is similar to T and, with the correct algebraic manipulations, we can
 * come to the conclusion that:
 *
 *     w_ba = 1 - w_ab - w_ac
 *
 * Since we know that w_ba >= 0, then w_ab + w_ac <= 1. Thus:
 *
 *     ----------------------------------------------------------
 *     | p = a + w_ab * ab + w_ac * ac is in the area of T iff: |
 *     | w_ab >= 0 and w_ac >= 0 and w_ab + w_ac <= 1           |
 *     ----------------------------------------------------------
 *
 * Proving backwards shouldn't be difficult.
 *
 * Vector p can also be written as:
 *
 *     p = (1 - w_ab - w_ba) * a + w_ab * b + w_ba * c
 *
 *
 * Vector that crosses a triangle
 * ------------------------------
 * Let T be the same triangle discussed above and let v be a vector such that:
 *
 *     v = x * a + y * b + z * c
 *     where x >= 0, y >= 0, z >= 0, and x + y + z > 0.
 *
 * It's geometrically easy to see that v crosses the triangle T. But that can
 * also be verified analytically.
 *
 * The vector v crosses the triangle T iff there's a positive alpha such that
 * alpha * v is in the area formed by T, so we need to prove that such value
 * exists. To find alpha, we solve the equation alpha * v = p, which will lead
 * us to the system, for the variables alpha, w_ab and w_ac:
 *
 *    alpha * x = 1 - w_ab - w_ac
 *    alpha * y = w_ab
 *    alpha * z = w_ac,
 *    where w_ab >= 0 and w_ac >= 0 and w_ab + w_ac <= 1
 *
 * That will lead to alpha = 1 / (x + y + z), w_ab = y / (x + y + b) and
 * w_ac = z / (x + y + z) and the following holds:
 *  - alpha does exist because x + y + z > 0.
 *  - w_ab >= 0 and w_ac >= 0 because y >= 0 and z >= 0 and x + y + z > 0.
 *  - 0 <= 1 - w_ab - w_ac <= 1 because 0 <= (y + z) / (x + y + z) <= 1.
 *
 * Thus:
 *
 *     ----------------------------------------------------------
 *     | v = x * a + y * b + z * c crosses T = (a, b, c), where |
 *     | a, b and c are linearly independent, iff:              |
 *     | x >= 0, y >= 0, z >= 0 and x + y + z > 0               |
 *     ----------------------------------------------------------
 *
 * Moreover:
 *  - if one of the coefficients is zero, then v crosses the edge formed by the
 *  vertices multiplied by the non-zero coefficients.
 *  - if two of the coefficients are zero, then v crosses the vertex multiplied
 *  by the non-zero coefficient.
 */

#include <assert.h>
#include "ch.h"
#include "vector3.h"
#include "matrix3.h"
#include "geodisicGrip.h"

/**
 * Number of sub-triangles for an icosahedron triangle.
 */
//static const int NUM_SUBTRIANGLES = 4;

/**
 * The representation of the neighbor umbrellas of T_0.
 *
 * The values for the neighbors of T_10 can be derived from the values for
 * T_0. How to find the correct values is explained on each member.
 *
 * Let T_0 = (a, b, c). Thus, 6 indexes can be used for this data
 * structure, so that:
 *  - index 0 represents the neighbor of T_0 with respect to (a, b).
 *  - index 1 represents the neighbor of T_0 with respect to (b, c).
 *  - index 2 represents the neighbor of T_0 with respect to (c, a).
 *  - index 3 represents the neighbor of T_10 with respect to (-a, -b).
 *  - index 4 represents the neighbor of T_10 with respect to (-b, -c).
 *  - index 5 represents the neighbor of T_10 with respect to (-c, -a).
 *
 * Those indexes are mapped to this array with index % 3.
 *
 * The edges are represented with pairs because the order of the vertices
 * matters to the order the triangles' indexes are defined - the order of
 * the umbrellas' vertices and components is convertioned to be with
 * respect to those pairs.
 */
struct neighbor_umbrella {
    /**
     * The umbrella's components. The value of #components[i] is the
     * icosahedron triangle index of the i-th component.
     *
     * In order to find the components for T_10, the following just finding
     * the index of the opposite triangle is enough. In other words,
     * (#components[i] + 10) % 20.
     */
    uint8_t components[5];
    /**
     * The fields with name in the format vi_cj are interpreted as the
     * following: vi_cj is the index of the vector, in the icosahedron
     * triangle pointed by #components[j], that matches the umbrella's i-th
     * vertex.
     *
     * The values don't change for T_10.
     */
    uint8_t v0_c0;
    uint8_t v1_c1;
    uint8_t v2_c1;
    uint8_t v4_c4;
    uint8_t v0_c4;
};

/* This was generated with
 * libraries/AP_Math/tools/geodesic_grid/geodesic_grid.py */
struct neighbor_umbrella _neighbor_umbrellas[3] = {
    {{ 9,  8,  7, 12, 14}, 1, 2, 0, 0, 2},
    {{ 1,  2,  4,  5,  3}, 0, 0, 2, 2, 0},
    {{16, 15, 13, 18, 17}, 2, 2, 0, 2, 1},
};

/* This was generated with
 * libraries/AP_Math/tools/geodesic_grid/geodesic_grid.py */
 /**
 * The inverses of the change-of-basis matrices for the icosahedron
 * triangles.
 *
 * The i-th matrix is the inverse of the change-of-basis matrix from
 * natural basis to the basis formed by T_i's vectors.
 */
MATRIX3 _inverses[10] = {
     {{{-0.309017f,  0.500000f,  0.190983f},
       { 0.000000f,  0.000000f, -0.618034f},
       {-0.309017f, -0.500000f,  0.190983f}}},
     
     {{{-0.190983f,  0.309017f, -0.500000f},
       {-0.500000f, -0.190983f,  0.309017f},
       { 0.309017f, -0.500000f, -0.190983f}}},
     
     {{{-0.618034f,  0.000000f,  0.000000f},
       { 0.190983f, -0.309017f, -0.500000f},
       { 0.190983f, -0.309017f,  0.500000f}}},
     
     {{{-0.500000f,  0.190983f, -0.309017f},
       { 0.000000f, -0.618034f,  0.000000f},
       { 0.500000f,  0.190983f, -0.309017f}}},
     
     {{{-0.190983f, -0.309017f, -0.500000f},
       {-0.190983f, -0.309017f,  0.500000f},
       { 0.618034f,  0.000000f,  0.000000f}}},
     
     {{{-0.309017f, -0.500000f, -0.190983f},
       { 0.190983f,  0.309017f, -0.500000f},
       { 0.500000f, -0.190983f,  0.309017f}}},
     
     {{{ 0.309017f, -0.500000f,  0.190983f},
       { 0.000000f,  0.000000f, -0.618034f},
       { 0.309017f,  0.500000f,  0.190983f}}},
     
     {{{ 0.190983f, -0.309017f, -0.500000f},
       { 0.500000f,  0.190983f,  0.309017f},
       {-0.309017f,  0.500000f, -0.190983f}}},
     
     {{{ 0.500000f, -0.190983f, -0.309017f},
       { 0.000000f,  0.618034f,  0.000000f},
       {-0.500000f, -0.190983f, -0.309017f}}},
     
     {{{ 0.309017f,  0.500000f, -0.190983f},
       {-0.500000f,  0.190983f,  0.309017f},
       {-0.190983f, -0.309017f, -0.500000f}}},
};

/* This was generated with
 * libraries/AP_Math/tools/geodesic_grid/geodesic_grid.py */
 /**
 * The inverses of the change-of-basis matrices for the middle triangles.
 *
 * The i-th matrix is the inverse of the change-of-basis matrix from
 * natural basis to the basis formed by T_i's middle triangle's vectors.
 */
MATRIX3 _mid_inverses[10] = {
     {{{-0.000000f,  1.000000f, -0.618034f},
       { 0.000000f, -1.000000f, -0.618034f},
       {-0.618034f,  0.000000f,  1.000000f}}},
     
     {{{-1.000000f,  0.618034f, -0.000000f},
       {-0.000000f, -1.000000f,  0.618034f},
       { 0.618034f, -0.000000f, -1.000000f}}},
     
     {{{-0.618034f, -0.000000f, -1.000000f},
       { 1.000000f, -0.618034f, -0.000000f},
       {-0.618034f,  0.000000f,  1.000000f}}},
     
     {{{-1.000000f, -0.618034f, -0.000000f},
       { 1.000000f, -0.618034f,  0.000000f},
       {-0.000000f,  1.000000f, -0.618034f}}},
     
     {{{-1.000000f, -0.618034f,  0.000000f},
       { 0.618034f,  0.000000f,  1.000000f},
       { 0.618034f,  0.000000f, -1.000000f}}},
     
     {{{-0.618034f, -0.000000f, -1.000000f},
       { 1.000000f,  0.618034f, -0.000000f},
       { 0.000000f, -1.000000f,  0.618034f}}},
     
     {{{ 0.000000f, -1.000000f, -0.618034f},
       { 0.000000f,  1.000000f, -0.618034f},
       { 0.618034f, -0.000000f,  1.000000f}}},
     
     {{{ 1.000000f, -0.618034f, -0.000000f},
       { 0.000000f,  1.000000f,  0.618034f},
       {-0.618034f,  0.000000f, -1.000000f}}},
     
     {{{ 1.000000f,  0.618034f, -0.000000f},
       {-1.000000f,  0.618034f,  0.000000f},
       { 0.000000f, -1.000000f, -0.618034f}}},
     
     {{{-0.000000f,  1.000000f,  0.618034f},
       {-1.000000f, -0.618034f, -0.000000f},
       { 0.618034f,  0.000000f, -1.000000f}}},
};

int section(VECTOR3 *v, bool inclusive)
{
    int i = _triangle_index(v, inclusive);
    if (i < 0) {
        return -1;
    }

    int j = _subtriangle_index(i, v, inclusive);
    if (j < 0) {
        return -1;
    }

    return 4 * i + j;
}

int _neighbor_umbrella_component(int idx, int comp_idx)
{
    if (idx < 3) {
        return _neighbor_umbrellas[idx].components[comp_idx];
    }
    return (_neighbor_umbrellas[idx % 3].components[comp_idx] + 10) % 20;
}

int _from_neighbor_umbrella(int idx,
                                     VECTOR3 *v,
                                     VECTOR3 *u,
                                     bool inclusive)
{
    /* The following comparisons between the umbrella's first and second
     * vertices' coefficients work for this algorithm because all vertices'
     * vectors are of the same length. */

    if (is_equal(u->v[0], u->v[1])) {
        /* If the coefficients of the first and second vertices are equal, then
         * v crosses the first component or the edge formed by the umbrella's
         * pivot and forth vertex. */
        int comp = _neighbor_umbrella_component(idx, 0);
        VECTOR3 w = matrix3_mul_vectro3(&_inverses[comp % 10], v);
        if (comp > 9) {
            w = vector3_neg(&w);
        }
        float x0 = w.v[_neighbor_umbrellas[idx % 3].v0_c0];
        if (is_zero(x0)) {
            if (!inclusive) {
                return -1;
            }
            return comp;
        } else if (x0 < 0) {
            if (!inclusive) {
                return -1;
            }
            return _neighbor_umbrella_component(idx, u->v[0] < u->v[1] ? 3 : 2);
        }

        return comp;
    }

    if (u->v[1] > u->v[0]) {
        /* If the coefficient of the second vertex is greater than the first
         * one's, then v crosses the first, second or third component. */
        int comp = _neighbor_umbrella_component(idx, 1);
        VECTOR3 w = matrix3_mul_vectro3(&_inverses[comp % 10], v);
        if (comp > 9) {
            w = vector3_neg(&w);
        }
        float x1 = w.v[_neighbor_umbrellas[idx % 3].v1_c1];
        float x2 = w.v[_neighbor_umbrellas[idx % 3].v2_c1];

        if (is_zero(x1)) {
            if (!inclusive) {
                return -1;
            }
            return _neighbor_umbrella_component(idx, x1 < 0 ? 2 : 1);
        } else if (x1 < 0) {
            return _neighbor_umbrella_component(idx, 2);
        }

        if (is_zero(x2)) {
            if (!inclusive) {
                return -1;
            }
            return _neighbor_umbrella_component(idx, x2 > 0 ? 1 : 0);
        } else if (x2 < 0) {
            return _neighbor_umbrella_component(idx, 0);
        }

        return comp;
    } else {
        /* If the coefficient of the second vertex is lesser than the first
         * one's, then v crosses the first, fourth or fifth component. */
        int comp = _neighbor_umbrella_component(idx, 4);
        VECTOR3 w = matrix3_mul_vectro3(&_inverses[comp % 10], v);
        if (comp > 9) {
            w = vector3_neg(&w);
        }
        float x4 = w.v[_neighbor_umbrellas[idx % 3].v4_c4];
        float x0 = w.v[_neighbor_umbrellas[idx % 3].v0_c4];

        if (is_zero(x4)) {
            if (!inclusive) {
                return -1;
            }
            return _neighbor_umbrella_component(idx, x4 < 0 ? 0 : 4);
        } else if (x4 < 0) {
            return _neighbor_umbrella_component(idx, 0);
        }

        if (is_zero(x0)) {
            if (!inclusive) {
                return -1;
            }
            return _neighbor_umbrella_component(idx, x0 > 0 ? 4 : 3);
        } else if (x0 < 0) {
            return _neighbor_umbrella_component(idx, 3);
        }

        return comp;
    }
}

int _triangle_index(VECTOR3 *v, bool inclusive)
{
    /* w holds the coordinates of v with respect to the basis comprised by the
     * vectors of T_i */
    VECTOR3 w = matrix3_mul_vectro3(&_inverses[0], v);
    int zero_count = 0;
    int balance = 0;
    int umbrella = -1;

    if (is_zero(w.v[0])) {
        zero_count++;
    } else if (w.v[0] > 0) {
        balance++;
    } else {
        balance--;
    }

    if (is_zero(w.v[1])) {
        zero_count++;
    } else if (w.v[1] > 0) {
        balance++;
    } else {
        balance--;
    }

    if (is_zero(w.v[2])) {
        zero_count++;
    } else if (w.v[2] > 0) {
        balance++;
    } else {
        balance--;
    }

    switch (balance) {
    case 3:
        /* All coefficients are positive, thus return the first triangle. */
        return 0;
    case -3:
        /* All coefficients are negative, which means that the coefficients for
         * -w are positive, thus return the first triangle's opposite. */
        return 10;
    case 2:
        /* Two coefficients are positive and one is zero, thus v crosses one of
         * the edges of the first triangle. */
        return inclusive ? 0 : -1;
    case -2:
        /* Analogous to the previous case, but for the opposite of the first
         * triangle. */
        return inclusive ? 10 : -1;
    case 1:
        /* There are two possible cases when balance is 1:
         *
         * 1) Two coefficients are zero, which means v crosses one of the
         * vertices of the first triangle.
         *
         * 2) Two coefficients are positive and one is negative. Let a and b be
         * vertices with positive coefficients and c the one with the negative
         * coefficient. That means that v crosses the triangle formed by a, b
         * and -c. The vector -c happens to be the 3-th vertex, with respect to
         * (a, b), of the first triangle's neighbor umbrella with respect to a
         * and b. Thus, v crosses one of the components of that umbrella. */
        if (zero_count == 2) {
            return inclusive ? 0 : -1;
        }

        if (!is_zero(w.v[0]) && w.v[0] < 0) {
            umbrella = 1;
        } else if (!is_zero(w.v[1]) && w.v[1] < 0) {
            umbrella = 2;
        } else {
            umbrella = 0;
        }

        break;
    case -1:
        /* Analogous to the previous case, but for the opposite of the first
         * triangle. */
        if (zero_count == 2) {
            return inclusive ? 10 : -1;
        }

        if (!is_zero(w.v[0]) && w.v[0] > 0) {
            umbrella = 4;
        } else if (!is_zero(w.v[1]) && w.v[1] > 0) {
            umbrella = 5;
        } else {
            umbrella = 3;
        }
        w = vector3_neg(&w);

        break;
    case 0:
        /* There are two possible cases when balance is 1:
         *
         * 1) The vector v is the null vector, which doesn't cross any section.
         *
         * 2) One coefficient is zero, another is positive and yet another is
         * negative. Let a, b and c be the respective vertices for those
         * coefficients, then the statements in case (2) for when balance is 1
         * are also valid here.
         */
        if (zero_count == 3) {
            return -1;
        }

        if (!is_zero(w.v[0]) && w.v[0] < 0) {
            umbrella = 1;
        } else if (!is_zero(w.v[1]) && w.v[1] < 0) {
            umbrella = 2;
        } else {
            umbrella = 0;
        }

        break;
    }

    assert(umbrella >= 0);

    switch (umbrella % 3) {
    case 0:
        w.v[2] = -w.v[2];
        break;
    case 1:
        w = vector3_new(w.v[1], w.v[2], -w.v[0]);
        break;
    case 2:
        w = vector3_new(w.v[2], w.v[0], -w.v[1]);
        break;
    }

    return _from_neighbor_umbrella(umbrella, v, &w, inclusive);
}

int _subtriangle_index(const unsigned int triangle_index,
                                        VECTOR3 *v,
                                        bool inclusive)
{
    /* w holds the coordinates of v with respect to the basis comprised by the
     * vectors of the middle triangle of T_i where i is triangle_index */
    VECTOR3 w = matrix3_mul_vectro3(&_mid_inverses[triangle_index % 10], v);
    if (triangle_index > 9) {
        w = vector3_neg(&w);
    }

    if ((is_zero(w.v[0]) || is_zero(w.v[1]) || is_zero(w.v[2])) && !inclusive) {
        return -1;
    }

    /* At this point, we know that v crosses the icosahedron triangle pointed
     * by triangle_index. Thus, we can geometrically see that if v doesn't
     * cross its middle triangle, then one of the coefficients will be negative
     * and the other ones positive. Let a and b be the non-negative
     * coefficients and c the negative one. In that case, v will cross the
     * triangle with vertices (a, b, -c). Since we know that v crosses the
     * icosahedron triangle and the only sub-triangle that contains the set of
     * points (seen as vectors) that cross the triangle (a, b, -c) is the
     * middle triangle's neighbor with respect to a and b, then that
     * sub-triangle is the one crossed by v. */
    if (!is_zero(w.v[0]) && w.v[0] < 0) {
        return 3;
    }
    if (!is_zero(w.v[1]) && w.v[1] < 0) {
        return 1;
    }
    if (!is_zero(w.v[2]) && w.v[2] < 0) {
        return 2;
    }

    /* If x >= 0 and y >= 0 and z >= 0, then v crosses the middle triangle. */
    return 0;
}

