/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Translational_drone_impl_dae_fun_jac_x_xdot_z_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_trans CASADI_PREFIX(trans)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

static const casadi_int casadi_s0[8] = {0, 1, 2, 6, 9, 12, 15, 19};
static const casadi_int casadi_s1[6] = {3, 10, 13, 16, 20, 23};
static const casadi_int casadi_s2[6] = {4, 7, 14, 17, 21, 24};
static const casadi_int casadi_s3[5] = {5, 8, 11, 18, 22};
static const casadi_int casadi_s4[38] = {10, 10, 0, 0, 0, 0, 5, 11, 17, 22, 23, 24, 25, 4, 5, 6, 7, 8, 3, 5, 6, 7, 8, 9, 3, 4, 6, 7, 8, 9, 3, 4, 5, 7, 8, 0, 1, 2};
static const casadi_int casadi_s5[38] = {10, 10, 0, 1, 2, 3, 6, 9, 12, 15, 19, 23, 25, 7, 8, 9, 4, 5, 6, 3, 5, 6, 3, 4, 6, 3, 4, 5, 3, 4, 5, 6, 3, 4, 5, 6, 4, 5};
static const casadi_int casadi_s6[23] = {10, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s7[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s8[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s9[3] = {0, 0, 0};
static const casadi_int casadi_s10[3] = {10, 0, 0};

/* Translational_drone_impl_dae_fun_jac_x_xdot_z:(i0[10],i1[10],i2[4],i3[],i4[])->(o0[10],o1[10x10,25nz],o2[10x10,10nz],o3[10x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, *w10=w+10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, *w26=w+35, *w27=w+45, *w28=w+70, *w31=w+77, *w32=w+85, *w35=w+93, *w36=w+99, *w38=w+105, *w39=w+110, *w40=w+115;
  /* #0: @0 = input[1][0] */
  w0 = arg[1] ? arg[1][0] : 0;
  /* #1: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #2: @2 = input[1][2] */
  w2 = arg[1] ? arg[1][2] : 0;
  /* #3: @3 = input[1][3] */
  w3 = arg[1] ? arg[1][3] : 0;
  /* #4: @4 = input[1][4] */
  w4 = arg[1] ? arg[1][4] : 0;
  /* #5: @5 = input[1][5] */
  w5 = arg[1] ? arg[1][5] : 0;
  /* #6: @6 = input[1][6] */
  w6 = arg[1] ? arg[1][6] : 0;
  /* #7: @7 = input[1][7] */
  w7 = arg[1] ? arg[1][7] : 0;
  /* #8: @8 = input[1][8] */
  w8 = arg[1] ? arg[1][8] : 0;
  /* #9: @9 = input[1][9] */
  w9 = arg[1] ? arg[1][9] : 0;
  /* #10: @10 = vertcat(@0, @1, @2, @3, @4, @5, @6, @7, @8, @9) */
  rr=w10;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  /* #11: @0 = input[0][7] */
  w0 = arg[0] ? arg[0][7] : 0;
  /* #12: @1 = input[0][8] */
  w1 = arg[0] ? arg[0][8] : 0;
  /* #13: @2 = input[0][9] */
  w2 = arg[0] ? arg[0][9] : 0;
  /* #14: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #15: @4 = input[2][1] */
  w4 = arg[2] ? arg[2][1] : 0;
  /* #16: @5 = input[0][4] */
  w5 = arg[0] ? arg[0][4] : 0;
  /* #17: @6 = (@4*@5) */
  w6  = (w4*w5);
  /* #18: @6 = (-@6) */
  w6 = (- w6 );
  /* #19: @7 = input[2][2] */
  w7 = arg[2] ? arg[2][2] : 0;
  /* #20: @8 = input[0][5] */
  w8 = arg[0] ? arg[0][5] : 0;
  /* #21: @9 = (@7*@8) */
  w9  = (w7*w8);
  /* #22: @6 = (@6-@9) */
  w6 -= w9;
  /* #23: @9 = input[2][3] */
  w9 = arg[2] ? arg[2][3] : 0;
  /* #24: @11 = input[0][6] */
  w11 = arg[0] ? arg[0][6] : 0;
  /* #25: @12 = (@9*@11) */
  w12  = (w9*w11);
  /* #26: @6 = (@6-@12) */
  w6 -= w12;
  /* #27: @6 = (@3*@6) */
  w6  = (w3*w6);
  /* #28: @12 = 0.5 */
  w12 = 5.0000000000000000e-01;
  /* #29: @13 = input[0][3] */
  w13 = arg[0] ? arg[0][3] : 0;
  /* #30: @14 = (@4*@13) */
  w14  = (w4*w13);
  /* #31: @15 = (@9*@8) */
  w15  = (w9*w8);
  /* #32: @14 = (@14+@15) */
  w14 += w15;
  /* #33: @15 = (@7*@11) */
  w15  = (w7*w11);
  /* #34: @14 = (@14-@15) */
  w14 -= w15;
  /* #35: @14 = (@12*@14) */
  w14  = (w12*w14);
  /* #36: @15 = 0.5 */
  w15 = 5.0000000000000000e-01;
  /* #37: @16 = (@7*@13) */
  w16  = (w7*w13);
  /* #38: @17 = (@9*@5) */
  w17  = (w9*w5);
  /* #39: @16 = (@16-@17) */
  w16 -= w17;
  /* #40: @17 = (@4*@11) */
  w17  = (w4*w11);
  /* #41: @16 = (@16+@17) */
  w16 += w17;
  /* #42: @16 = (@15*@16) */
  w16  = (w15*w16);
  /* #43: @17 = 0.5 */
  w17 = 5.0000000000000000e-01;
  /* #44: @18 = (@9*@13) */
  w18  = (w9*w13);
  /* #45: @19 = (@7*@5) */
  w19  = (w7*w5);
  /* #46: @18 = (@18+@19) */
  w18 += w19;
  /* #47: @19 = (@4*@8) */
  w19  = (w4*w8);
  /* #48: @18 = (@18-@19) */
  w18 -= w19;
  /* #49: @18 = (@17*@18) */
  w18  = (w17*w18);
  /* #50: @19 = (@13*@8) */
  w19  = (w13*w8);
  /* #51: @20 = (@5*@11) */
  w20  = (w5*w11);
  /* #52: @19 = (@19+@20) */
  w19 += w20;
  /* #53: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #54: @20 = input[2][0] */
  w20 = arg[2] ? arg[2][0] : 0;
  /* #55: @19 = (@19*@20) */
  w19 *= w20;
  /* #56: @21 = (@8*@11) */
  w21  = (w8*w11);
  /* #57: @22 = (@13*@5) */
  w22  = (w13*w5);
  /* #58: @21 = (@21-@22) */
  w21 -= w22;
  /* #59: @21 = (2.*@21) */
  w21 = (2.* w21 );
  /* #60: @21 = (@21*@20) */
  w21 *= w20;
  /* #61: @22 = 1 */
  w22 = 1.;
  /* #62: @23 = (2.*@5) */
  w23 = (2.* w5 );
  /* #63: @24 = (@23*@5) */
  w24  = (w23*w5);
  /* #64: @22 = (@22-@24) */
  w22 -= w24;
  /* #65: @24 = (2.*@8) */
  w24 = (2.* w8 );
  /* #66: @25 = (@24*@8) */
  w25  = (w24*w8);
  /* #67: @22 = (@22-@25) */
  w22 -= w25;
  /* #68: @22 = (@22*@20) */
  w22 *= w20;
  /* #69: @25 = 9.81 */
  w25 = 9.8100000000000005e+00;
  /* #70: @22 = (@22-@25) */
  w22 -= w25;
  /* #71: @26 = vertcat(@0, @1, @2, @6, @14, @16, @18, @19, @21, @22) */
  rr=w26;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w6;
  *rr++ = w14;
  *rr++ = w16;
  *rr++ = w18;
  *rr++ = w19;
  *rr++ = w21;
  *rr++ = w22;
  /* #72: @10 = (@10-@26) */
  for (i=0, rr=w10, cs=w26; i<10; ++i) (*rr++) -= (*cs++);
  /* #73: output[0][0] = @10 */
  casadi_copy(w10, 10, res[0]);
  /* #74: @27 = zeros(10x10,25nz) */
  casadi_clear(w27, 25);
  /* #75: @28 = ones(10x1,7nz) */
  casadi_fill(w28, 7, 1.);
  /* #76: {NULL, NULL, NULL, @0, NULL, NULL, NULL, @1, @2, @6} = vertsplit(@28) */
  w0 = w28[3];
  w1 = w28[4];
  w2 = w28[5];
  w6 = w28[6];
  /* #77: @29 = 00 */
  /* #78: @14 = (@4*@0) */
  w14  = (w4*w0);
  /* #79: @14 = (@12*@14) */
  w14  = (w12*w14);
  /* #80: @16 = (@7*@0) */
  w16  = (w7*w0);
  /* #81: @16 = (@15*@16) */
  w16  = (w15*w16);
  /* #82: @18 = (@9*@0) */
  w18  = (w9*w0);
  /* #83: @18 = (@17*@18) */
  w18  = (w17*w18);
  /* #84: @19 = (@8*@0) */
  w19  = (w8*w0);
  /* #85: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #86: @19 = (@20*@19) */
  w19  = (w20*w19);
  /* #87: @0 = (@5*@0) */
  w0  = (w5*w0);
  /* #88: @0 = (-@0) */
  w0 = (- w0 );
  /* #89: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #90: @0 = (@20*@0) */
  w0  = (w20*w0);
  /* #91: @30 = 00 */
  /* #92: @31 = vertcat(@1, @2, @6, @29, @14, @16, @18, @19, @0, @30) */
  rr=w31;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w6;
  *rr++ = w14;
  *rr++ = w16;
  *rr++ = w18;
  *rr++ = w19;
  *rr++ = w0;
  /* #93: @31 = (-@31) */
  for (i=0, rr=w31, cs=w31; i<8; ++i) *rr++ = (- *cs++ );
  /* #94: @32 = @31[:8] */
  for (rr=w32, ss=w31+0; ss!=w31+8; ss+=1) *rr++ = *ss;
  /* #95: (@27[0, 1, 2, 6, 9, 12, 15, 19] = @32) */
  for (cii=casadi_s0, rr=w27, ss=w32; cii!=casadi_s0+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #96: @29 = 00 */
  /* #97: @30 = 00 */
  /* #98: @33 = 00 */
  /* #99: @1 = ones(10x1,1nz) */
  w1 = 1.;
  /* #100: {NULL, NULL, NULL, NULL, @2, NULL, NULL, NULL, NULL, NULL} = vertsplit(@1) */
  w2 = w1;
  /* #101: @1 = (@4*@2) */
  w1  = (w4*w2);
  /* #102: @1 = (@3*@1) */
  w1  = (w3*w1);
  /* #103: @1 = (-@1) */
  w1 = (- w1 );
  /* #104: @34 = 00 */
  /* #105: @6 = (@9*@2) */
  w6  = (w9*w2);
  /* #106: @6 = (@15*@6) */
  w6  = (w15*w6);
  /* #107: @6 = (-@6) */
  w6 = (- w6 );
  /* #108: @14 = (@7*@2) */
  w14  = (w7*w2);
  /* #109: @14 = (@17*@14) */
  w14  = (w17*w14);
  /* #110: @16 = (@11*@2) */
  w16  = (w11*w2);
  /* #111: @16 = (2.*@16) */
  w16 = (2.* w16 );
  /* #112: @16 = (@20*@16) */
  w16  = (w20*w16);
  /* #113: @18 = (@13*@2) */
  w18  = (w13*w2);
  /* #114: @18 = (-@18) */
  w18 = (- w18 );
  /* #115: @18 = (2.*@18) */
  w18 = (2.* w18 );
  /* #116: @18 = (@20*@18) */
  w18  = (w20*w18);
  /* #117: @19 = (2.*@2) */
  w19 = (2.* w2 );
  /* #118: @19 = (@5*@19) */
  w19  = (w5*w19);
  /* #119: @23 = (@23*@2) */
  w23 *= w2;
  /* #120: @19 = (@19+@23) */
  w19 += w23;
  /* #121: @19 = (@20*@19) */
  w19  = (w20*w19);
  /* #122: @19 = (-@19) */
  w19 = (- w19 );
  /* #123: @35 = vertcat(@29, @30, @33, @1, @34, @6, @14, @16, @18, @19) */
  rr=w35;
  *rr++ = w1;
  *rr++ = w6;
  *rr++ = w14;
  *rr++ = w16;
  *rr++ = w18;
  *rr++ = w19;
  /* #124: @35 = (-@35) */
  for (i=0, rr=w35, cs=w35; i<6; ++i) *rr++ = (- *cs++ );
  /* #125: @36 = @35[:6] */
  for (rr=w36, ss=w35+0; ss!=w35+6; ss+=1) *rr++ = *ss;
  /* #126: (@27[3, 10, 13, 16, 20, 23] = @36) */
  for (cii=casadi_s1, rr=w27, ss=w36; cii!=casadi_s1+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #127: @29 = 00 */
  /* #128: @30 = 00 */
  /* #129: @33 = 00 */
  /* #130: @1 = ones(10x1,1nz) */
  w1 = 1.;
  /* #131: {NULL, NULL, NULL, NULL, NULL, @6, NULL, NULL, NULL, NULL} = vertsplit(@1) */
  w6 = w1;
  /* #132: @1 = (@7*@6) */
  w1  = (w7*w6);
  /* #133: @1 = (@3*@1) */
  w1  = (w3*w1);
  /* #134: @1 = (-@1) */
  w1 = (- w1 );
  /* #135: @14 = (@9*@6) */
  w14  = (w9*w6);
  /* #136: @14 = (@12*@14) */
  w14  = (w12*w14);
  /* #137: @34 = 00 */
  /* #138: @16 = (@4*@6) */
  w16  = (w4*w6);
  /* #139: @17 = (@17*@16) */
  w17 *= w16;
  /* #140: @17 = (-@17) */
  w17 = (- w17 );
  /* #141: @13 = (@13*@6) */
  w13 *= w6;
  /* #142: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #143: @13 = (@20*@13) */
  w13  = (w20*w13);
  /* #144: @11 = (@11*@6) */
  w11 *= w6;
  /* #145: @11 = (2.*@11) */
  w11 = (2.* w11 );
  /* #146: @11 = (@20*@11) */
  w11  = (w20*w11);
  /* #147: @16 = (2.*@6) */
  w16 = (2.* w6 );
  /* #148: @16 = (@8*@16) */
  w16  = (w8*w16);
  /* #149: @24 = (@24*@6) */
  w24 *= w6;
  /* #150: @16 = (@16+@24) */
  w16 += w24;
  /* #151: @16 = (@20*@16) */
  w16  = (w20*w16);
  /* #152: @16 = (-@16) */
  w16 = (- w16 );
  /* #153: @36 = vertcat(@29, @30, @33, @1, @14, @34, @17, @13, @11, @16) */
  rr=w36;
  *rr++ = w1;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w13;
  *rr++ = w11;
  *rr++ = w16;
  /* #154: @36 = (-@36) */
  for (i=0, rr=w36, cs=w36; i<6; ++i) *rr++ = (- *cs++ );
  /* #155: @35 = @36[:6] */
  for (rr=w35, ss=w36+0; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #156: (@27[4, 7, 14, 17, 21, 24] = @35) */
  for (cii=casadi_s2, rr=w27, ss=w35; cii!=casadi_s2+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #157: @29 = 00 */
  /* #158: @30 = 00 */
  /* #159: @33 = 00 */
  /* #160: @1 = ones(10x1,1nz) */
  w1 = 1.;
  /* #161: {NULL, NULL, NULL, NULL, NULL, NULL, @14, NULL, NULL, NULL} = vertsplit(@1) */
  w14 = w1;
  /* #162: @9 = (@9*@14) */
  w9 *= w14;
  /* #163: @3 = (@3*@9) */
  w3 *= w9;
  /* #164: @3 = (-@3) */
  w3 = (- w3 );
  /* #165: @7 = (@7*@14) */
  w7 *= w14;
  /* #166: @12 = (@12*@7) */
  w12 *= w7;
  /* #167: @12 = (-@12) */
  w12 = (- w12 );
  /* #168: @4 = (@4*@14) */
  w4 *= w14;
  /* #169: @15 = (@15*@4) */
  w15 *= w4;
  /* #170: @34 = 00 */
  /* #171: @5 = (@5*@14) */
  w5 *= w14;
  /* #172: @5 = (2.*@5) */
  w5 = (2.* w5 );
  /* #173: @5 = (@20*@5) */
  w5  = (w20*w5);
  /* #174: @8 = (@8*@14) */
  w8 *= w14;
  /* #175: @8 = (2.*@8) */
  w8 = (2.* w8 );
  /* #176: @20 = (@20*@8) */
  w20 *= w8;
  /* #177: @37 = 00 */
  /* #178: @38 = vertcat(@29, @30, @33, @3, @12, @15, @34, @5, @20, @37) */
  rr=w38;
  *rr++ = w3;
  *rr++ = w12;
  *rr++ = w15;
  *rr++ = w5;
  *rr++ = w20;
  /* #179: @38 = (-@38) */
  for (i=0, rr=w38, cs=w38; i<5; ++i) *rr++ = (- *cs++ );
  /* #180: @39 = @38[:5] */
  for (rr=w39, ss=w38+0; ss!=w38+5; ss+=1) *rr++ = *ss;
  /* #181: (@27[5, 8, 11, 18, 22] = @39) */
  for (cii=casadi_s3, rr=w27, ss=w39; cii!=casadi_s3+5; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #182: @40 = @27' */
  casadi_trans(w27,casadi_s5, w40, casadi_s4, iw);
  /* #183: output[1][0] = @40 */
  casadi_copy(w40, 25, res[1]);
  /* #184: @10 = zeros(10x10,10nz) */
  casadi_clear(w10, 10);
  /* #185: @26 = ones(10x1) */
  casadi_fill(w26, 10, 1.);
  /* #186: (@10[:10] = @26) */
  for (rr=w10+0, ss=w26; rr!=w10+10; rr+=1) *rr = *ss++;
  /* #187: @26 = @10' */
  casadi_trans(w10,casadi_s6, w26, casadi_s6, iw);
  /* #188: output[2][0] = @26 */
  casadi_copy(w26, 10, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_fun_jac_x_xdot_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_fun_jac_x_xdot_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_impl_dae_fun_jac_x_xdot_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_fun_jac_x_xdot_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_fun_jac_x_xdot_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_fun_jac_x_xdot_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s7;
    case 1: return casadi_s7;
    case 2: return casadi_s8;
    case 3: return casadi_s9;
    case 4: return casadi_s9;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_fun_jac_x_xdot_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s7;
    case 1: return casadi_s4;
    case 2: return casadi_s6;
    case 3: return casadi_s10;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 15;
  if (sz_res) *sz_res = 14;
  if (sz_iw) *sz_iw = 11;
  if (sz_w) *sz_w = 140;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
