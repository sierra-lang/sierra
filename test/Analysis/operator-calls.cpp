// RUN: %clang_cc1 -analyze -analyzer-checker=core,experimental.core,debug.ExprInspection -analyzer-ipa=inlining -verify %s
void clang_analyzer_eval(bool);

struct X0 { };
bool operator==(const X0&, const X0&);

// PR7287
struct test { int a[2]; };

void t2() {
  test p = {{1,2}};
  test q;
  q = p;
}

bool PR7287(X0 a, X0 b) {
  return operator==(a, b);
}


// Inlining non-static member operators mistakenly treated 'this' as the first
// argument for a while.

struct IntComparable {
  bool operator==(int x) const {
    return x == 0;
  }
};

void testMemberOperator(IntComparable B) {
  clang_analyzer_eval(B == 0); // expected-warning{{TRUE}}
}
