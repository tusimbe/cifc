#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>
#include <float.h>
#include <string.h>
#include "ch.h"
#include "vector3.h"
#include "matrix3.h"

static void vector3_add_test(void **state) {
    (void)state;
    
    VECTOR3 a, b;
    VECTOR3 result, *rep;

    a.v[0] = 1;
    a.v[1] = 2;
    a.v[2] = 3;

    b.v[0] = 5;
    b.v[1] = 10;
    b.v[2] = 15;

    result = vector3_add(&a, &b);

    assert_float_equal(result.v[0], 6, FLT_EPSILON);
    assert_float_equal(result.v[1], 12, FLT_EPSILON);
    assert_float_equal(result.v[2], 18, FLT_EPSILON);

    a = vector3_add(&a, &b);

    assert_float_equal(a.v[0], 6, FLT_EPSILON);
    assert_float_equal(a.v[1], 12, FLT_EPSILON);
    assert_float_equal(a.v[2], 18, FLT_EPSILON);

    rep = &result;

    *rep = vector3_add(&a, &b);

    assert_float_equal(result.v[0], 11, FLT_EPSILON);
    assert_float_equal(result.v[1], 22, FLT_EPSILON);
    assert_float_equal(result.v[2], 33, FLT_EPSILON);
}

static void matrix3_mul_vectro3_test(void **state) {
    (void)state;

    MATRIX3 mtrx;
    VECTOR3 vct, res, *resp;

    mtrx.m[0][0] = 10;
    mtrx.m[0][1] = 24;
    mtrx.m[0][2] = 4;

    mtrx.m[1][0] = 4;
    mtrx.m[1][1] = 67;
    mtrx.m[1][2] = 123;

    mtrx.m[2][0] = 15;
    mtrx.m[2][1] = 1;
    mtrx.m[2][2] = 0;

    vct.v[0] = 1;
    vct.v[1] = 2;
    vct.v[2] = 3;

    res = matrix3_mul_vectro3(&mtrx, &vct);

    assert_float_equal(res.v[0], 70, FLT_EPSILON);
    assert_float_equal(res.v[1], 507, FLT_EPSILON);
    assert_float_equal(res.v[2], 17, FLT_EPSILON);

    memset(&res, 0, sizeof(VECTOR3));
    resp = &res;

    *resp = matrix3_mul_vectro3(&mtrx, &vct);

    assert_float_equal(res.v[0], 70, FLT_EPSILON);
    assert_float_equal(res.v[1], 507, FLT_EPSILON);
    assert_float_equal(res.v[2], 17, FLT_EPSILON);

    vct = matrix3_mul_vectro3(&mtrx, &vct);
    assert_float_equal(vct.v[0], 70, FLT_EPSILON);
    assert_float_equal(vct.v[1], 507, FLT_EPSILON);
    assert_float_equal(vct.v[2], 17, FLT_EPSILON);

}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(vector3_add_test),
        cmocka_unit_test(matrix3_mul_vectro3_test),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
