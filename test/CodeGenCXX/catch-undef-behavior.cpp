// RUN: %clang_cc1 -fsanitize=signed-integer-overflow,integer-divide-by-zero,float-divide-by-zero,shift,unreachable,return,vla-bound,alignment,null,vptr,object-size,float-cast-overflow,bool,enum -emit-llvm %s -o - -triple x86_64-linux-gnu | FileCheck %s

struct S {
  double d;
  int a, b;
  virtual int f();
};

// CHECK: @_Z17reference_binding
void reference_binding(int *p, S *q) {
  // C++ core issue 453: If an lvalue to which a reference is directly bound
  // designates neither an existing object or function of an appropriate type,
  // nor a region of storage of suitable size and alignment to contain an object
  // of the reference's type, the behavior is undefined.

  // CHECK: icmp ne {{.*}}, null

  // CHECK: %[[SIZE:.*]] = call i64 @llvm.objectsize.i64
  // CHECK-NEXT: icmp uge i64 %[[SIZE]], 4

  // CHECK: %[[PTRINT:.*]] = ptrtoint
  // CHECK-NEXT: %[[MISALIGN:.*]] = and i64 %[[PTRINT]], 3
  // CHECK-NEXT: icmp eq i64 %[[MISALIGN]], 0
  int &r = *p;

  // A reference is not required to refer to an object within its lifetime.
  // CHECK-NOT: __ubsan_handle_dynamic_type_cache_miss
  S &r2 = *q;
}

// CHECK: @_Z13member_access
void member_access(S *p) {
  // (1a) Check 'p' is appropriately sized and aligned for member access.

  // CHECK: icmp ne {{.*}}, null

  // CHECK: %[[SIZE:.*]] = call i64 @llvm.objectsize.i64
  // CHECK-NEXT: icmp uge i64 %[[SIZE]], 24

  // CHECK: %[[PTRINT:.*]] = ptrtoint
  // CHECK-NEXT: %[[MISALIGN:.*]] = and i64 %[[PTRINT]], 7
  // CHECK-NEXT: icmp eq i64 %[[MISALIGN]], 0

  // (1b) Check that 'p' actually points to an 'S'.

  // CHECK: %[[VPTRADDR:.*]] = bitcast {{.*}} to i64*
  // CHECK-NEXT: %[[VPTR:.*]] = load i64* %[[VPTRADDR]]
  //
  // hash_16_bytes:
  //
  // If this number changes, it indicates that either the mangled name of ::S
  // has changed, or that LLVM's hashing function has changed. The latter case
  // is OK if the hashing function is still stable.
  //
  // The two hash values are for 64- and 32-bit Clang binaries, respectively.
  // FIXME: We should produce a 64-bit value either way.
  //
  // CHECK-NEXT: xor i64 {{-4030275160588942838|2562089159}}, %[[VPTR]]
  // CHECK-NEXT: mul i64 {{.*}}, -7070675565921424023
  // CHECK-NEXT: lshr i64 {{.*}}, 47
  // CHECK-NEXT: xor i64
  // CHECK-NEXT: xor i64 %[[VPTR]]
  // CHECK-NEXT: mul i64 {{.*}}, -7070675565921424023
  // CHECK-NEXT: lshr i64 {{.*}}, 47
  // CHECK-NEXT: xor i64
  // CHECK-NEXT: %[[HASH:.*]] = mul i64 {{.*}}, -7070675565921424023
  //
  // Check the hash against the table:
  //
  // CHECK-NEXT: %[[IDX:.*]] = and i64 %{{.*}}, 127
  // CHECK-NEXT: getelementptr inbounds [128 x i64]* @__ubsan_vptr_type_cache, i32 0, i64 %[[IDX]]
  // CHECK-NEXT: %[[CACHEVAL:.*]] = load i64*
  // CHECK-NEXT: icmp eq i64 %[[CACHEVAL]], %[[HASH]]
  // CHECK-NEXT: br i1

  // CHECK: call void @__ubsan_handle_dynamic_type_cache_miss({{.*}}, i64 %{{.*}}, i64 %[[HASH]])
  // CHECK-NOT: unreachable
  // CHECK: {{.*}}:

  // (2) Check 'p->b' is appropriately sized and aligned for a load.

  // FIXME: Suppress this in the trivial case of a member access, because we
  // know we've just checked the member access expression itself.

  // CHECK: %[[SIZE:.*]] = call i64 @llvm.objectsize.i64
  // CHECK-NEXT: icmp uge i64 %[[SIZE]], 4

  // CHECK: %[[PTRINT:.*]] = ptrtoint
  // CHECK-NEXT: %[[MISALIGN:.*]] = and i64 %[[PTRINT]], 3
  // CHECK-NEXT: icmp eq i64 %[[MISALIGN]], 0
  int k = p->b;

  // (3a) Check 'p' is appropriately sized and aligned for member function call.

  // CHECK: icmp ne {{.*}}, null

  // CHECK: %[[SIZE:.*]] = call i64 @llvm.objectsize.i64
  // CHECK-NEXT: icmp uge i64 %[[SIZE]], 24

  // CHECK: %[[PTRINT:.*]] = ptrtoint
  // CHECK-NEXT: %[[MISALIGN:.*]] = and i64 %[[PTRINT]], 7
  // CHECK-NEXT: icmp eq i64 %[[MISALIGN]], 0

  // (3b) Check that 'p' actually points to an 'S'

  // CHECK: load i64*
  // CHECK-NEXT: xor i64 {{-4030275160588942838|2562089159}},
  // [...]
  // CHECK: getelementptr inbounds [128 x i64]* @__ubsan_vptr_type_cache, i32 0, i64 %
  // CHECK: br i1
  // CHECK: call void @__ubsan_handle_dynamic_type_cache_miss({{.*}}, i64 %{{.*}}, i64 %{{.*}})
  // CHECK-NOT: unreachable
  // CHECK: {{.*}}:

  k = p->f();
}

// CHECK: @_Z12lsh_overflow
int lsh_overflow(int a, int b) {
  // CHECK: %[[INBOUNDS:.*]] = icmp ule i32 %[[RHS:.*]], 31
  // CHECK-NEXT: br i1 %[[INBOUNDS]]

  // CHECK: %[[SHIFTED_OUT_WIDTH:.*]] = sub nuw nsw i32 31, %[[RHS]]
  // CHECK-NEXT: %[[SHIFTED_OUT:.*]] = lshr i32 %[[LHS:.*]], %[[SHIFTED_OUT_WIDTH]]

  // This is present for C++11 but not for C: C++ core issue 1457 allows a '1'
  // to be shifted into the sign bit, but not out of it.
  // CHECK-NEXT: %[[SHIFTED_OUT_NOT_SIGN:.*]] = lshr i32 %[[SHIFTED_OUT]], 1

  // CHECK-NEXT: %[[NO_OVERFLOW:.*]] = icmp eq i32 %[[SHIFTED_OUT_NOT_SIGN]], 0
  // CHECK-NEXT: br i1 %[[NO_OVERFLOW]]

  // CHECK: %[[RET:.*]] = shl i32 %[[LHS]], %[[RHS]]
  // CHECK-NEXT: ret i32 %[[RET]]
  return a << b;
}

// CHECK: @_Z9no_return
int no_return() {
  // CHECK:      call void @__ubsan_handle_missing_return(i8* bitcast ({{.*}}* @{{.*}} to i8*)) noreturn nounwind
  // CHECK-NEXT: unreachable
}

// CHECK: @_Z9sour_bool
bool sour_bool(bool *p) {
  // CHECK: %[[OK:.*]] = icmp ule i8 {{.*}}, 1
  // CHECK: br i1 %[[OK]]
  // CHECK: call void @__ubsan_handle_load_invalid_value(i8* bitcast ({{.*}}), i64 {{.*}})
  return *p;
}

enum E1 { e1a = 0, e1b = 127 } e1;
enum E2 { e2a = -1, e2b = 64 } e2;
enum E3 { e3a = (1u << 31) - 1 } e3;

// CHECK: @_Z14bad_enum_value
int bad_enum_value() {
  // CHECK: %[[E1:.*]] = icmp ule i32 {{.*}}, 127
  // CHECK: br i1 %[[E1]]
  // CHECK: call void @__ubsan_handle_load_invalid_value(
  int a = e1;

  // CHECK: %[[E2HI:.*]] = icmp sle i32 {{.*}}, 127
  // CHECK: %[[E2LO:.*]] = icmp sge i32 {{.*}}, -128
  // CHECK: %[[E2:.*]] = and i1 %[[E2HI]], %[[E2LO]]
  // CHECK: br i1 %[[E2]]
  // CHECK: call void @__ubsan_handle_load_invalid_value(
  int b = e2;

  // CHECK: %[[E3:.*]] = icmp ule i32 {{.*}}, 2147483647
  // CHECK: br i1 %[[E3]]
  // CHECK: call void @__ubsan_handle_load_invalid_value(
  int c = e3;
  return a + b + c;
}
