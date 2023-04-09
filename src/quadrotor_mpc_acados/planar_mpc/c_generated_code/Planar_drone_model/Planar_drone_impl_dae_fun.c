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
  #define CASADI_PREFIX(ID) Planar_drone_impl_dae_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* Planar_drone_impl_dae_fun:(i0[6],i1[6],i2[2],i3[],i4[])->(o0[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr;
  const casadi_real *cs;
  casadi_real w0, w1, w2, w3, w4, w5, *w6=w+6, w7, w8, *w9=w+14;
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
  /* #6: @6 = vertcat(@0, @1, @2, @3, @4, @5) */
  rr=w6;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #7: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #8: @1 = input[0][4] */
  w1 = arg[0] ? arg[0][4] : 0;
  /* #9: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #10: @3 = input[2][0] */
  w3 = arg[2] ? arg[2][0] : 0;
  /* #11: @4 = 0.01475 */
  w4 = 1.4749999999999999e-02;
  /* #12: @4 = (@3/@4) */
  w4  = (w3/w4);
  /* #13: @5 = input[0][2] */
  w5 = arg[0] ? arg[0][2] : 0;
  /* #14: @7 = sin(@5) */
  w7 = sin( w5 );
  /* #15: @4 = (@4*@7) */
  w4 *= w7;
  /* #16: @4 = (-@4) */
  w4 = (- w4 );
  /* #17: @7 = -9.81 */
  w7 = -9.8100000000000005e+00;
  /* #18: @8 = 0.01475 */
  w8 = 1.4749999999999999e-02;
  /* #19: @3 = (@3/@8) */
  w3 /= w8;
  /* #20: @5 = cos(@5) */
  w5 = cos( w5 );
  /* #21: @3 = (@3*@5) */
  w3 *= w5;
  /* #22: @7 = (@7+@3) */
  w7 += w3;
  /* #23: @3 = input[2][1] */
  w3 = arg[2] ? arg[2][1] : 0;
  /* #24: @5 = 1.65717e-05 */
  w5 = 1.6571710000000000e-05;
  /* #25: @3 = (@3/@5) */
  w3 /= w5;
  /* #26: @9 = vertcat(@0, @1, @2, @4, @7, @3) */
  rr=w9;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w4;
  *rr++ = w7;
  *rr++ = w3;
  /* #27: @6 = (@6-@9) */
  for (i=0, rr=w6, cs=w9; i<6; ++i) (*rr++) -= (*cs++);
  /* #28: output[0][0] = @6 */
  casadi_copy(w6, 6, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_impl_dae_fun_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_impl_dae_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real Planar_drone_impl_dae_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_impl_dae_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_impl_dae_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_impl_dae_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_impl_dae_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 20;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
