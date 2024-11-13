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
  #define CASADI_PREFIX(ID) Hovering_drone_expl_vde_forw_ ## ID
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
#define casadi_densify CASADI_PREFIX(densify)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

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

#define CASADI_CAST(x,y) ((x) y)

void casadi_densify(const casadi_real* x, const casadi_int* sp_x, casadi_real* y, casadi_int tr) {
  casadi_int nrow_x, ncol_x, i, el;
  const casadi_int *colind_x, *row_x;
  if (!y) return;
  nrow_x = sp_x[0]; ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x+ncol_x+3;
  casadi_clear(y, nrow_x*ncol_x);
  if (!x) return;
  if (tr) {
    for (i=0; i<ncol_x; ++i) {
      for (el=colind_x[i]; el!=colind_x[i+1]; ++el) {
        y[i + row_x[el]*ncol_x] = CASADI_CAST(casadi_real, *x++);
      }
    }
  } else {
    for (i=0; i<ncol_x; ++i) {
      for (el=colind_x[i]; el!=colind_x[i+1]; ++el) {
        y[row_x[el]] = CASADI_CAST(casadi_real, *x++);
      }
      y += nrow_x;
    }
  }
}

static const casadi_int casadi_s0[5] = {2, 1, 0, 1, 1};
static const casadi_int casadi_s1[5] = {2, 1, 0, 1, 0};
static const casadi_int casadi_s2[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s3[9] = {2, 2, 0, 2, 4, 0, 1, 0, 1};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[3] = {0, 0, 0};
static const casadi_int casadi_s6[7] = {2, 2, 0, 1, 2, 0, 0};

/* Hovering_drone_expl_vde_forw:(i0[2],i1[2x2],i2[2],i3,i4[])->(o0[2],o1[2x2,2nz],o2[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+2, *w3=w+6, *w4=w+8;
  /* #0: @0 = input[0][1] */
  w0 = arg[0] ? arg[0][1] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = 9.81 */
  w1 = 9.8100000000000005e+00;
  /* #4: @0 = (@0-@1) */
  w0 -= w1;
  /* #5: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #6: @2 = input[1][0] */
  casadi_copy(arg[1], 4, w2);
  /* #7: {@3, @4} = horzsplit(@2) */
  casadi_copy(w2, 2, w3);
  casadi_copy(w2+2, 2, w4);
  /* #8: {NULL, @0} = vertsplit(@3) */
  w0 = w3[1];
  /* #9: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #10: {NULL, @0} = vertsplit(@4) */
  w0 = w4[1];
  /* #11: output[1][1] = @0 */
  if (res[1]) res[1][1] = w0;
  /* #12: @0 = zeros(1x2,1nz) */
  w0 = 0.;
  /* #13: @1 = 1 */
  w1 = 1.;
  /* #14: (@0[0] = @1) */
  for (rr=(&w0)+0, ss=(&w1); rr!=(&w0)+1; rr+=1) *rr = *ss++;
  /* #15: @0 = @0' */
  /* #16: @4 = dense(@0) */
  casadi_densify((&w0), casadi_s0, w4, 0);
  /* #17: @3 = input[2][0] */
  casadi_copy(arg[2], 2, w3);
  /* #18: {NULL, @0} = vertsplit(@3) */
  w0 = w3[1];
  /* #19: @5 = 00 */
  /* #20: @1 = vertcat(@0, @5) */
  rr=(&w1);
  *rr++ = w0;
  /* #21: @3 = dense(@1) */
  casadi_densify((&w1), casadi_s1, w3, 0);
  /* #22: @4 = (@4+@3) */
  for (i=0, rr=w4, cs=w3; i<2; ++i) (*rr++) += (*cs++);
  /* #23: output[2][0] = @4 */
  casadi_copy(w4, 2, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Hovering_drone_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Hovering_drone_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Hovering_drone_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Hovering_drone_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Hovering_drone_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Hovering_drone_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Hovering_drone_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void Hovering_drone_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Hovering_drone_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Hovering_drone_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real Hovering_drone_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Hovering_drone_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Hovering_drone_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Hovering_drone_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s2;
    case 3: return casadi_s4;
    case 4: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Hovering_drone_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s6;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Hovering_drone_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 10;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif