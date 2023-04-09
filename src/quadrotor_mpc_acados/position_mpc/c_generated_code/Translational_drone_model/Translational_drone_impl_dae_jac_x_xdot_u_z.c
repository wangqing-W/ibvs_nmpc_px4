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

static const casadi_int casadi_s0[12] = {6, 6, 0, 0, 0, 0, 1, 2, 3, 0, 1, 2};
static const casadi_int casadi_s1[12] = {6, 6, 0, 1, 2, 3, 3, 3, 3, 3, 4, 5};
static const casadi_int casadi_s2[15] = {6, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[3] = {2, 7, 11};
static const casadi_int casadi_s4[3] = {3, 8, 12};
static const casadi_int casadi_s5[21] = {6, 5, 0, 3, 5, 8, 11, 13, 3, 4, 5, 3, 4, 3, 4, 5, 3, 4, 5, 3, 4};
static const casadi_int casadi_s6[22] = {5, 6, 0, 0, 0, 0, 5, 10, 13, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 2, 3};
static const casadi_int casadi_s7[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s8[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s9[3] = {0, 0, 0};
static const casadi_int casadi_s10[3] = {6, 0, 0};

/* Translational_drone_impl_dae_jac_x_xdot_u_z:(i0[6],i1[6],i2[5],i3[],i4[])->(o0[6x6,3nz],o1[6x6,6nz],o2[6x5,13nz],o3[6x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real *w0=w+0, *w1=w+3, w2, w3, w4, *w8=w+12, *w9=w+15, *w10=w+18, *w11=w+24, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, *w24=w+48, *w25=w+50, *w26=w+52;
  /* #0: @0 = zeros(6x6,3nz) */
  casadi_clear(w0, 3);
  /* #1: @1 = ones(6x1) */
  casadi_fill(w1, 6, 1.);
  /* #2: {NULL, NULL, NULL, @2, @3, @4} = vertsplit(@1) */
  w2 = w1[3];
  w3 = w1[4];
  w4 = w1[5];
  /* #3: @5 = 00 */
  /* #4: @6 = 00 */
  /* #5: @7 = 00 */
  /* #6: @8 = vertcat(@2, @3, @4, @5, @6, @7) */
  rr=w8;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #7: @8 = (-@8) */
  for (i=0, rr=w8, cs=w8; i<3; ++i) *rr++ = (- *cs++ );
  /* #8: @9 = @8[:3] */
  for (rr=w9, ss=w8+0; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #9: (@0[:3] = @9) */
  for (rr=w0+0, ss=w9; rr!=w0+3; rr+=1) *rr = *ss++;
  /* #10: @9 = @0' */
  casadi_trans(w0,casadi_s1, w9, casadi_s0, iw);
  /* #11: output[0][0] = @9 */
  casadi_copy(w9, 3, res[0]);
  /* #12: @1 = zeros(6x6,6nz) */
  casadi_clear(w1, 6);
  /* #13: @10 = ones(6x1) */
  casadi_fill(w10, 6, 1.);
  /* #14: (@1[:6] = @10) */
  for (rr=w1+0, ss=w10; rr!=w1+6; rr+=1) *rr = *ss++;
  /* #15: @10 = @1' */
  casadi_trans(w1,casadi_s2, w10, casadi_s2, iw);
  /* #16: output[1][0] = @10 */
  casadi_copy(w10, 6, res[1]);
  /* #17: @11 = zeros(5x6,13nz) */
  casadi_clear(w11, 13);
  /* #18: @5 = 00 */
  /* #19: @6 = 00 */
  /* #20: @7 = 00 */
  /* #21: @2 = 37.037 */
  w2 = 3.7037037037037038e+01;
  /* #22: @3 = input[2][1] */
  w3 = arg[2] ? arg[2][1] : 0;
  /* #23: @4 = input[2][3] */
  w4 = arg[2] ? arg[2][3] : 0;
  /* #24: @12 = (@3*@4) */
  w12  = (w3*w4);
  /* #25: @13 = input[2][2] */
  w13 = arg[2] ? arg[2][2] : 0;
  /* #26: @14 = input[2][4] */
  w14 = arg[2] ? arg[2][4] : 0;
  /* #27: @15 = (@13*@14) */
  w15  = (w13*w14);
  /* #28: @12 = (@12+@15) */
  w12 += w15;
  /* #29: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #30: @15 = ones(5x1,1nz) */
  w15 = 1.;
  /* #31: {@16, NULL, NULL, NULL, NULL} = vertsplit(@15) */
  w16 = w15;
  /* #32: @12 = (@12*@16) */
  w12 *= w16;
  /* #33: @12 = (@2*@12) */
  w12  = (w2*w12);
  /* #34: @15 = 37.037 */
  w15 = 3.7037037037037038e+01;
  /* #35: @17 = (@4*@14) */
  w17  = (w4*w14);
  /* #36: @18 = (@3*@13) */
  w18  = (w3*w13);
  /* #37: @17 = (@17-@18) */
  w17 -= w18;
  /* #38: @17 = (2.*@17) */
  w17 = (2.* w17 );
  /* #39: @17 = (@17*@16) */
  w17 *= w16;
  /* #40: @17 = (@15*@17) */
  w17  = (w15*w17);
  /* #41: @18 = 37.037 */
  w18 = 3.7037037037037038e+01;
  /* #42: @19 = 1 */
  w19 = 1.;
  /* #43: @20 = (2.*@13) */
  w20 = (2.* w13 );
  /* #44: @21 = (@20*@13) */
  w21  = (w20*w13);
  /* #45: @19 = (@19-@21) */
  w19 -= w21;
  /* #46: @21 = (2.*@4) */
  w21 = (2.* w4 );
  /* #47: @22 = (@21*@4) */
  w22  = (w21*w4);
  /* #48: @19 = (@19-@22) */
  w19 -= w22;
  /* #49: @19 = (@19*@16) */
  w19 *= w16;
  /* #50: @19 = (@18*@19) */
  w19  = (w18*w19);
  /* #51: @9 = vertcat(@5, @6, @7, @12, @17, @19) */
  rr=w9;
  *rr++ = w12;
  *rr++ = w17;
  *rr++ = w19;
  /* #52: @9 = (-@9) */
  for (i=0, rr=w9, cs=w9; i<3; ++i) *rr++ = (- *cs++ );
  /* #53: @0 = @9[:3] */
  for (rr=w0, ss=w9+0; ss!=w9+3; ss+=1) *rr++ = *ss;
  /* #54: (@11[:15:5] = @0) */
  for (rr=w11+0, ss=w0; rr!=w11+15; rr+=5) *rr = *ss++;
  /* #55: @5 = 00 */
  /* #56: @6 = 00 */
  /* #57: @7 = 00 */
  /* #58: @12 = input[2][0] */
  w12 = arg[2] ? arg[2][0] : 0;
  /* #59: @17 = ones(5x1,1nz) */
  w17 = 1.;
  /* #60: {NULL, @19, NULL, NULL, NULL} = vertsplit(@17) */
  w19 = w17;
  /* #61: @17 = (@4*@19) */
  w17  = (w4*w19);
  /* #62: @17 = (2.*@17) */
  w17 = (2.* w17 );
  /* #63: @17 = (@12*@17) */
  w17  = (w12*w17);
  /* #64: @17 = (@2*@17) */
  w17  = (w2*w17);
  /* #65: @19 = (@13*@19) */
  w19  = (w13*w19);
  /* #66: @19 = (-@19) */
  w19 = (- w19 );
  /* #67: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #68: @19 = (@12*@19) */
  w19  = (w12*w19);
  /* #69: @19 = (@15*@19) */
  w19  = (w15*w19);
  /* #70: @23 = 00 */
  /* #71: @24 = vertcat(@5, @6, @7, @17, @19, @23) */
  rr=w24;
  *rr++ = w17;
  *rr++ = w19;
  /* #72: @24 = (-@24) */
  for (i=0, rr=w24, cs=w24; i<2; ++i) *rr++ = (- *cs++ );
  /* #73: @25 = @24[:2] */
  for (rr=w25, ss=w24+0; ss!=w24+2; ss+=1) *rr++ = *ss;
  /* #74: (@11[1:11:5] = @25) */
  for (rr=w11+1, ss=w25; rr!=w11+11; rr+=5) *rr = *ss++;
  /* #75: @5 = 00 */
  /* #76: @6 = 00 */
  /* #77: @7 = 00 */
  /* #78: @17 = ones(5x1,1nz) */
  w17 = 1.;
  /* #79: {NULL, NULL, @19, NULL, NULL} = vertsplit(@17) */
  w19 = w17;
  /* #80: @17 = (@14*@19) */
  w17  = (w14*w19);
  /* #81: @17 = (2.*@17) */
  w17 = (2.* w17 );
  /* #82: @17 = (@12*@17) */
  w17  = (w12*w17);
  /* #83: @17 = (@2*@17) */
  w17  = (w2*w17);
  /* #84: @16 = (@3*@19) */
  w16  = (w3*w19);
  /* #85: @16 = (-@16) */
  w16 = (- w16 );
  /* #86: @16 = (2.*@16) */
  w16 = (2.* w16 );
  /* #87: @16 = (@12*@16) */
  w16  = (w12*w16);
  /* #88: @16 = (@15*@16) */
  w16  = (w15*w16);
  /* #89: @22 = (2.*@19) */
  w22 = (2.* w19 );
  /* #90: @22 = (@13*@22) */
  w22  = (w13*w22);
  /* #91: @20 = (@20*@19) */
  w20 *= w19;
  /* #92: @22 = (@22+@20) */
  w22 += w20;
  /* #93: @22 = (@12*@22) */
  w22  = (w12*w22);
  /* #94: @22 = (@18*@22) */
  w22  = (w18*w22);
  /* #95: @22 = (-@22) */
  w22 = (- w22 );
  /* #96: @0 = vertcat(@5, @6, @7, @17, @16, @22) */
  rr=w0;
  *rr++ = w17;
  *rr++ = w16;
  *rr++ = w22;
  /* #97: @0 = (-@0) */
  for (i=0, rr=w0, cs=w0; i<3; ++i) *rr++ = (- *cs++ );
  /* #98: @9 = @0[:3] */
  for (rr=w9, ss=w0+0; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #99: (@11[2, 7, 11] = @9) */
  for (cii=casadi_s3, rr=w11, ss=w9; cii!=casadi_s3+3; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #100: @5 = 00 */
  /* #101: @6 = 00 */
  /* #102: @7 = 00 */
  /* #103: @17 = ones(5x1,1nz) */
  w17 = 1.;
  /* #104: {NULL, NULL, NULL, @16, NULL} = vertsplit(@17) */
  w16 = w17;
  /* #105: @3 = (@3*@16) */
  w3 *= w16;
  /* #106: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #107: @3 = (@12*@3) */
  w3  = (w12*w3);
  /* #108: @3 = (@2*@3) */
  w3  = (w2*w3);
  /* #109: @14 = (@14*@16) */
  w14 *= w16;
  /* #110: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #111: @14 = (@12*@14) */
  w14  = (w12*w14);
  /* #112: @14 = (@15*@14) */
  w14  = (w15*w14);
  /* #113: @17 = (2.*@16) */
  w17 = (2.* w16 );
  /* #114: @17 = (@4*@17) */
  w17  = (w4*w17);
  /* #115: @21 = (@21*@16) */
  w21 *= w16;
  /* #116: @17 = (@17+@21) */
  w17 += w21;
  /* #117: @17 = (@12*@17) */
  w17  = (w12*w17);
  /* #118: @18 = (@18*@17) */
  w18 *= w17;
  /* #119: @18 = (-@18) */
  w18 = (- w18 );
  /* #120: @9 = vertcat(@5, @6, @7, @3, @14, @18) */
  rr=w9;
  *rr++ = w3;
  *rr++ = w14;
  *rr++ = w18;
  /* #121: @9 = (-@9) */
  for (i=0, rr=w9, cs=w9; i<3; ++i) *rr++ = (- *cs++ );
  /* #122: @0 = @9[:3] */
  for (rr=w0, ss=w9+0; ss!=w9+3; ss+=1) *rr++ = *ss;
  /* #123: (@11[3, 8, 12] = @0) */
  for (cii=casadi_s4, rr=w11, ss=w0; cii!=casadi_s4+3; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #124: @5 = 00 */
  /* #125: @6 = 00 */
  /* #126: @7 = 00 */
  /* #127: @3 = ones(5x1,1nz) */
  w3 = 1.;
  /* #128: {NULL, NULL, NULL, NULL, @14} = vertsplit(@3) */
  w14 = w3;
  /* #129: @13 = (@13*@14) */
  w13 *= w14;
  /* #130: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #131: @13 = (@12*@13) */
  w13  = (w12*w13);
  /* #132: @2 = (@2*@13) */
  w2 *= w13;
  /* #133: @4 = (@4*@14) */
  w4 *= w14;
  /* #134: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #135: @12 = (@12*@4) */
  w12 *= w4;
  /* #136: @15 = (@15*@12) */
  w15 *= w12;
  /* #137: @23 = 00 */
  /* #138: @25 = vertcat(@5, @6, @7, @2, @15, @23) */
  rr=w25;
  *rr++ = w2;
  *rr++ = w15;
  /* #139: @25 = (-@25) */
  for (i=0, rr=w25, cs=w25; i<2; ++i) *rr++ = (- *cs++ );
  /* #140: @24 = @25[:2] */
  for (rr=w24, ss=w25+0; ss!=w25+2; ss+=1) *rr++ = *ss;
  /* #141: (@11[4:14:5] = @24) */
  for (rr=w11+4, ss=w24; rr!=w11+14; rr+=5) *rr = *ss++;
  /* #142: @26 = @11' */
  casadi_trans(w11,casadi_s6, w26, casadi_s5, iw);
  /* #143: output[2][0] = @26 */
  casadi_copy(w26, 13, res[2]);
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
    case 0: return casadi_s7;
    case 1: return casadi_s7;
    case 2: return casadi_s8;
    case 3: return casadi_s9;
    case 4: return casadi_s9;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s2;
    case 2: return casadi_s5;
    case 3: return casadi_s10;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 10;
  if (sz_iw) *sz_iw = 7;
  if (sz_w) *sz_w = 65;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
