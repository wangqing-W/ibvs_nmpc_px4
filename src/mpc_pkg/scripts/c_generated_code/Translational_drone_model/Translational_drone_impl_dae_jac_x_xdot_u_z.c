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
  #define CASADI_PREFIX(ID) Translational_drone_impl_dae_jac_x_xdot_u_z_ ## ID
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
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
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

static const casadi_int casadi_s0[8] = {0, 1, 2, 6, 9, 12, 15, 19};
static const casadi_int casadi_s1[6] = {3, 10, 13, 16, 20, 23};
static const casadi_int casadi_s2[6] = {4, 7, 14, 17, 21, 24};
static const casadi_int casadi_s3[5] = {5, 8, 11, 18, 22};
static const casadi_int casadi_s4[38] = {10, 10, 0, 0, 0, 0, 5, 11, 17, 22, 23, 24, 25, 4, 5, 6, 7, 8, 3, 5, 6, 7, 8, 9, 3, 4, 6, 7, 8, 9, 3, 4, 5, 7, 8, 0, 1, 2};
static const casadi_int casadi_s5[38] = {10, 10, 0, 1, 2, 3, 6, 9, 12, 15, 19, 23, 25, 7, 8, 9, 4, 5, 6, 3, 5, 6, 3, 4, 6, 3, 4, 5, 3, 4, 5, 6, 3, 4, 5, 6, 4, 5};
static const casadi_int casadi_s6[23] = {10, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s7[7] = {0, 3, 6, 9, 12, 13, 14};
static const casadi_int casadi_s8[22] = {10, 4, 0, 3, 7, 11, 15, 7, 8, 9, 3, 4, 5, 6, 3, 4, 5, 6, 3, 4, 5, 6};
static const casadi_int casadi_s9[28] = {4, 10, 0, 0, 0, 0, 3, 6, 9, 12, 13, 14, 15, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 0, 0, 0};
static const casadi_int casadi_s10[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s11[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s12[3] = {0, 0, 0};
static const casadi_int casadi_s13[3] = {10, 0, 0};

/* Translational_drone_impl_dae_jac_x_xdot_u_z:(i0[10],i1[10],i2[4],i3[],i4[])->(o0[10x10,25nz],o1[10x10,10nz],o2[10x4,15nz],o3[10x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real *w0=w+0, *w1=w+25, w2, w3, w4, w5, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, *w21=w+49, *w22=w+57, w25, w26, w27, *w28=w+68, *w29=w+74, w30, *w32=w+81, *w33=w+86, *w34=w+91, *w35=w+116, *w36=w+126, *w37=w+136, *w38=w+151, *w39=w+153, *w41=w+160, *w42=w+164, *w43=w+168;
  /* #0: @0 = zeros(10x10,25nz) */
  casadi_clear(w0, 25);
  /* #1: @1 = ones(10x1,7nz) */
  casadi_fill(w1, 7, 1.);
  /* #2: {NULL, NULL, NULL, @2, NULL, NULL, NULL, @3, @4, @5} = vertsplit(@1) */
  w2 = w1[3];
  w3 = w1[4];
  w4 = w1[5];
  w5 = w1[6];
  /* #3: @6 = 00 */
  /* #4: @7 = 0.5 */
  w7 = 5.0000000000000000e-01;
  /* #5: @8 = input[2][1] */
  w8 = arg[2] ? arg[2][1] : 0;
  /* #6: @9 = (@8*@2) */
  w9  = (w8*w2);
  /* #7: @9 = (@7*@9) */
  w9  = (w7*w9);
  /* #8: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #9: @11 = input[2][2] */
  w11 = arg[2] ? arg[2][2] : 0;
  /* #10: @12 = (@11*@2) */
  w12  = (w11*w2);
  /* #11: @12 = (@10*@12) */
  w12  = (w10*w12);
  /* #12: @13 = 0.5 */
  w13 = 5.0000000000000000e-01;
  /* #13: @14 = input[2][3] */
  w14 = arg[2] ? arg[2][3] : 0;
  /* #14: @15 = (@14*@2) */
  w15  = (w14*w2);
  /* #15: @15 = (@13*@15) */
  w15  = (w13*w15);
  /* #16: @16 = input[2][0] */
  w16 = arg[2] ? arg[2][0] : 0;
  /* #17: @17 = input[0][5] */
  w17 = arg[0] ? arg[0][5] : 0;
  /* #18: @18 = (@17*@2) */
  w18  = (w17*w2);
  /* #19: @18 = (2.*@18) */
  w18 = (2.* w18 );
  /* #20: @18 = (@16*@18) */
  w18  = (w16*w18);
  /* #21: @19 = input[0][4] */
  w19 = arg[0] ? arg[0][4] : 0;
  /* #22: @2 = (@19*@2) */
  w2  = (w19*w2);
  /* #23: @2 = (-@2) */
  w2 = (- w2 );
  /* #24: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #25: @2 = (@16*@2) */
  w2  = (w16*w2);
  /* #26: @20 = 00 */
  /* #27: @21 = vertcat(@3, @4, @5, @6, @9, @12, @15, @18, @2, @20) */
  rr=w21;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w15;
  *rr++ = w18;
  *rr++ = w2;
  /* #28: @21 = (-@21) */
  for (i=0, rr=w21, cs=w21; i<8; ++i) *rr++ = (- *cs++ );
  /* #29: @22 = @21[:8] */
  for (rr=w22, ss=w21+0; ss!=w21+8; ss+=1) *rr++ = *ss;
  /* #30: (@0[0, 1, 2, 6, 9, 12, 15, 19] = @22) */
  for (cii=casadi_s0, rr=w0, ss=w22; cii!=casadi_s0+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #31: @6 = 00 */
  /* #32: @20 = 00 */
  /* #33: @23 = 00 */
  /* #34: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #35: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #36: {NULL, NULL, NULL, NULL, @5, NULL, NULL, NULL, NULL, NULL} = vertsplit(@4) */
  w5 = w4;
  /* #37: @4 = (@8*@5) */
  w4  = (w8*w5);
  /* #38: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #39: @4 = (-@4) */
  w4 = (- w4 );
  /* #40: @24 = 00 */
  /* #41: @9 = (@14*@5) */
  w9  = (w14*w5);
  /* #42: @9 = (@10*@9) */
  w9  = (w10*w9);
  /* #43: @9 = (-@9) */
  w9 = (- w9 );
  /* #44: @12 = (@11*@5) */
  w12  = (w11*w5);
  /* #45: @12 = (@13*@12) */
  w12  = (w13*w12);
  /* #46: @15 = input[0][6] */
  w15 = arg[0] ? arg[0][6] : 0;
  /* #47: @18 = (@15*@5) */
  w18  = (w15*w5);
  /* #48: @18 = (2.*@18) */
  w18 = (2.* w18 );
  /* #49: @18 = (@16*@18) */
  w18  = (w16*w18);
  /* #50: @2 = input[0][3] */
  w2 = arg[0] ? arg[0][3] : 0;
  /* #51: @25 = (@2*@5) */
  w25  = (w2*w5);
  /* #52: @25 = (-@25) */
  w25 = (- w25 );
  /* #53: @25 = (2.*@25) */
  w25 = (2.* w25 );
  /* #54: @25 = (@16*@25) */
  w25  = (w16*w25);
  /* #55: @26 = (2.*@5) */
  w26 = (2.* w5 );
  /* #56: @26 = (@19*@26) */
  w26  = (w19*w26);
  /* #57: @27 = (2.*@19) */
  w27 = (2.* w19 );
  /* #58: @5 = (@27*@5) */
  w5  = (w27*w5);
  /* #59: @26 = (@26+@5) */
  w26 += w5;
  /* #60: @26 = (@16*@26) */
  w26  = (w16*w26);
  /* #61: @26 = (-@26) */
  w26 = (- w26 );
  /* #62: @28 = vertcat(@6, @20, @23, @4, @24, @9, @12, @18, @25, @26) */
  rr=w28;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w18;
  *rr++ = w25;
  *rr++ = w26;
  /* #63: @28 = (-@28) */
  for (i=0, rr=w28, cs=w28; i<6; ++i) *rr++ = (- *cs++ );
  /* #64: @29 = @28[:6] */
  for (rr=w29, ss=w28+0; ss!=w28+6; ss+=1) *rr++ = *ss;
  /* #65: (@0[3, 10, 13, 16, 20, 23] = @29) */
  for (cii=casadi_s1, rr=w0, ss=w29; cii!=casadi_s1+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #66: @6 = 00 */
  /* #67: @20 = 00 */
  /* #68: @23 = 00 */
  /* #69: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #70: {NULL, NULL, NULL, NULL, NULL, @9, NULL, NULL, NULL, NULL} = vertsplit(@4) */
  w9 = w4;
  /* #71: @4 = (@11*@9) */
  w4  = (w11*w9);
  /* #72: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #73: @4 = (-@4) */
  w4 = (- w4 );
  /* #74: @12 = (@14*@9) */
  w12  = (w14*w9);
  /* #75: @12 = (@7*@12) */
  w12  = (w7*w12);
  /* #76: @24 = 00 */
  /* #77: @18 = (@8*@9) */
  w18  = (w8*w9);
  /* #78: @18 = (@13*@18) */
  w18  = (w13*w18);
  /* #79: @18 = (-@18) */
  w18 = (- w18 );
  /* #80: @25 = (@2*@9) */
  w25  = (w2*w9);
  /* #81: @25 = (2.*@25) */
  w25 = (2.* w25 );
  /* #82: @25 = (@16*@25) */
  w25  = (w16*w25);
  /* #83: @26 = (@15*@9) */
  w26  = (w15*w9);
  /* #84: @26 = (2.*@26) */
  w26 = (2.* w26 );
  /* #85: @26 = (@16*@26) */
  w26  = (w16*w26);
  /* #86: @5 = (2.*@9) */
  w5 = (2.* w9 );
  /* #87: @5 = (@17*@5) */
  w5  = (w17*w5);
  /* #88: @30 = (2.*@17) */
  w30 = (2.* w17 );
  /* #89: @9 = (@30*@9) */
  w9  = (w30*w9);
  /* #90: @5 = (@5+@9) */
  w5 += w9;
  /* #91: @5 = (@16*@5) */
  w5  = (w16*w5);
  /* #92: @5 = (-@5) */
  w5 = (- w5 );
  /* #93: @29 = vertcat(@6, @20, @23, @4, @12, @24, @18, @25, @26, @5) */
  rr=w29;
  *rr++ = w4;
  *rr++ = w12;
  *rr++ = w18;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w5;
  /* #94: @29 = (-@29) */
  for (i=0, rr=w29, cs=w29; i<6; ++i) *rr++ = (- *cs++ );
  /* #95: @28 = @29[:6] */
  for (rr=w28, ss=w29+0; ss!=w29+6; ss+=1) *rr++ = *ss;
  /* #96: (@0[4, 7, 14, 17, 21, 24] = @28) */
  for (cii=casadi_s2, rr=w0, ss=w28; cii!=casadi_s2+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #97: @6 = 00 */
  /* #98: @20 = 00 */
  /* #99: @23 = 00 */
  /* #100: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #101: {NULL, NULL, NULL, NULL, NULL, NULL, @12, NULL, NULL, NULL} = vertsplit(@4) */
  w12 = w4;
  /* #102: @14 = (@14*@12) */
  w14 *= w12;
  /* #103: @14 = (@3*@14) */
  w14  = (w3*w14);
  /* #104: @14 = (-@14) */
  w14 = (- w14 );
  /* #105: @11 = (@11*@12) */
  w11 *= w12;
  /* #106: @11 = (@7*@11) */
  w11  = (w7*w11);
  /* #107: @11 = (-@11) */
  w11 = (- w11 );
  /* #108: @8 = (@8*@12) */
  w8 *= w12;
  /* #109: @8 = (@10*@8) */
  w8  = (w10*w8);
  /* #110: @24 = 00 */
  /* #111: @4 = (@19*@12) */
  w4  = (w19*w12);
  /* #112: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #113: @4 = (@16*@4) */
  w4  = (w16*w4);
  /* #114: @12 = (@17*@12) */
  w12  = (w17*w12);
  /* #115: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #116: @16 = (@16*@12) */
  w16 *= w12;
  /* #117: @31 = 00 */
  /* #118: @32 = vertcat(@6, @20, @23, @14, @11, @8, @24, @4, @16, @31) */
  rr=w32;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w4;
  *rr++ = w16;
  /* #119: @32 = (-@32) */
  for (i=0, rr=w32, cs=w32; i<5; ++i) *rr++ = (- *cs++ );
  /* #120: @33 = @32[:5] */
  for (rr=w33, ss=w32+0; ss!=w32+5; ss+=1) *rr++ = *ss;
  /* #121: (@0[5, 8, 11, 18, 22] = @33) */
  for (cii=casadi_s3, rr=w0, ss=w33; cii!=casadi_s3+5; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #122: @34 = @0' */
  casadi_trans(w0,casadi_s5, w34, casadi_s4, iw);
  /* #123: output[0][0] = @34 */
  casadi_copy(w34, 25, res[0]);
  /* #124: @35 = zeros(10x10,10nz) */
  casadi_clear(w35, 10);
  /* #125: @36 = ones(10x1) */
  casadi_fill(w36, 10, 1.);
  /* #126: (@35[:10] = @36) */
  for (rr=w35+0, ss=w36; rr!=w35+10; rr+=1) *rr = *ss++;
  /* #127: @36 = @35' */
  casadi_trans(w35,casadi_s6, w36, casadi_s6, iw);
  /* #128: output[1][0] = @36 */
  casadi_copy(w36, 10, res[1]);
  /* #129: @37 = zeros(4x10,15nz) */
  casadi_clear(w37, 15);
  /* #130: @6 = 00 */
  /* #131: @20 = 00 */
  /* #132: @23 = 00 */
  /* #133: @38 = ones(4x1,2nz) */
  casadi_fill(w38, 2, 1.);
  /* #134: {@14, @11, NULL, NULL} = vertsplit(@38) */
  w14 = w38[0];
  w11 = w38[1];
  /* #135: @8 = (@19*@11) */
  w8  = (w19*w11);
  /* #136: @8 = (@3*@8) */
  w8  = (w3*w8);
  /* #137: @8 = (-@8) */
  w8 = (- w8 );
  /* #138: @4 = (@2*@11) */
  w4  = (w2*w11);
  /* #139: @4 = (@7*@4) */
  w4  = (w7*w4);
  /* #140: @16 = (@15*@11) */
  w16  = (w15*w11);
  /* #141: @16 = (@10*@16) */
  w16  = (w10*w16);
  /* #142: @11 = (@17*@11) */
  w11  = (w17*w11);
  /* #143: @11 = (@13*@11) */
  w11  = (w13*w11);
  /* #144: @11 = (-@11) */
  w11 = (- w11 );
  /* #145: @12 = (@2*@17) */
  w12  = (w2*w17);
  /* #146: @18 = (@19*@15) */
  w18  = (w19*w15);
  /* #147: @12 = (@12+@18) */
  w12 += w18;
  /* #148: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #149: @12 = (@12*@14) */
  w12 *= w14;
  /* #150: @18 = (@17*@15) */
  w18  = (w17*w15);
  /* #151: @25 = (@2*@19) */
  w25  = (w2*w19);
  /* #152: @18 = (@18-@25) */
  w18 -= w25;
  /* #153: @18 = (2.*@18) */
  w18 = (2.* w18 );
  /* #154: @18 = (@18*@14) */
  w18 *= w14;
  /* #155: @25 = 1 */
  w25 = 1.;
  /* #156: @27 = (@27*@19) */
  w27 *= w19;
  /* #157: @25 = (@25-@27) */
  w25 -= w27;
  /* #158: @30 = (@30*@17) */
  w30 *= w17;
  /* #159: @25 = (@25-@30) */
  w25 -= w30;
  /* #160: @25 = (@25*@14) */
  w25 *= w14;
  /* #161: @1 = vertcat(@6, @20, @23, @8, @4, @16, @11, @12, @18, @25) */
  rr=w1;
  *rr++ = w8;
  *rr++ = w4;
  *rr++ = w16;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w18;
  *rr++ = w25;
  /* #162: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<7; ++i) *rr++ = (- *cs++ );
  /* #163: @39 = @1[:7] */
  for (rr=w39, ss=w1+0; ss!=w1+7; ss+=1) *rr++ = *ss;
  /* #164: (@37[0, 3, 6, 9, 12, 13, 14] = @39) */
  for (cii=casadi_s7, rr=w37, ss=w39; cii!=casadi_s7+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #165: @6 = 00 */
  /* #166: @20 = 00 */
  /* #167: @23 = 00 */
  /* #168: @8 = ones(4x1,1nz) */
  w8 = 1.;
  /* #169: {NULL, NULL, @4, NULL} = vertsplit(@8) */
  w4 = w8;
  /* #170: @8 = (@17*@4) */
  w8  = (w17*w4);
  /* #171: @8 = (@3*@8) */
  w8  = (w3*w8);
  /* #172: @8 = (-@8) */
  w8 = (- w8 );
  /* #173: @16 = (@15*@4) */
  w16  = (w15*w4);
  /* #174: @16 = (@7*@16) */
  w16  = (w7*w16);
  /* #175: @16 = (-@16) */
  w16 = (- w16 );
  /* #176: @11 = (@2*@4) */
  w11  = (w2*w4);
  /* #177: @11 = (@10*@11) */
  w11  = (w10*w11);
  /* #178: @4 = (@19*@4) */
  w4  = (w19*w4);
  /* #179: @4 = (@13*@4) */
  w4  = (w13*w4);
  /* #180: @24 = 00 */
  /* #181: @31 = 00 */
  /* #182: @40 = 00 */
  /* #183: @41 = vertcat(@6, @20, @23, @8, @16, @11, @4, @24, @31, @40) */
  rr=w41;
  *rr++ = w8;
  *rr++ = w16;
  *rr++ = w11;
  *rr++ = w4;
  /* #184: @41 = (-@41) */
  for (i=0, rr=w41, cs=w41; i<4; ++i) *rr++ = (- *cs++ );
  /* #185: @42 = @41[:4] */
  for (rr=w42, ss=w41+0; ss!=w41+4; ss+=1) *rr++ = *ss;
  /* #186: (@37[1:13:3] = @42) */
  for (rr=w37+1, ss=w42; rr!=w37+13; rr+=3) *rr = *ss++;
  /* #187: @6 = 00 */
  /* #188: @20 = 00 */
  /* #189: @23 = 00 */
  /* #190: @8 = ones(4x1,1nz) */
  w8 = 1.;
  /* #191: {NULL, NULL, NULL, @16} = vertsplit(@8) */
  w16 = w8;
  /* #192: @15 = (@15*@16) */
  w15 *= w16;
  /* #193: @3 = (@3*@15) */
  w3 *= w15;
  /* #194: @3 = (-@3) */
  w3 = (- w3 );
  /* #195: @17 = (@17*@16) */
  w17 *= w16;
  /* #196: @7 = (@7*@17) */
  w7 *= w17;
  /* #197: @19 = (@19*@16) */
  w19 *= w16;
  /* #198: @10 = (@10*@19) */
  w10 *= w19;
  /* #199: @10 = (-@10) */
  w10 = (- w10 );
  /* #200: @2 = (@2*@16) */
  w2 *= w16;
  /* #201: @13 = (@13*@2) */
  w13 *= w2;
  /* #202: @24 = 00 */
  /* #203: @31 = 00 */
  /* #204: @40 = 00 */
  /* #205: @42 = vertcat(@6, @20, @23, @3, @7, @10, @13, @24, @31, @40) */
  rr=w42;
  *rr++ = w3;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w13;
  /* #206: @42 = (-@42) */
  for (i=0, rr=w42, cs=w42; i<4; ++i) *rr++ = (- *cs++ );
  /* #207: @41 = @42[:4] */
  for (rr=w41, ss=w42+0; ss!=w42+4; ss+=1) *rr++ = *ss;
  /* #208: (@37[2:14:3] = @41) */
  for (rr=w37+2, ss=w41; rr!=w37+14; rr+=3) *rr = *ss++;
  /* #209: @43 = @37' */
  casadi_trans(w37,casadi_s9, w43, casadi_s8, iw);
  /* #210: output[2][0] = @43 */
  casadi_copy(w43, 15, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_jac_x_xdot_u_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_jac_x_xdot_u_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_impl_dae_jac_x_xdot_u_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_jac_x_xdot_u_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_jac_x_xdot_u_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s10;
    case 1: return casadi_s10;
    case 2: return casadi_s11;
    case 3: return casadi_s12;
    case 4: return casadi_s12;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s6;
    case 2: return casadi_s8;
    case 3: return casadi_s13;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 15;
  if (sz_res) *sz_res = 14;
  if (sz_iw) *sz_iw = 11;
  if (sz_w) *sz_w = 183;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
