// RUN: %clang_cc1 -analyze -analyzer-checker=core,unix.Malloc,debug.ExprInspection -verify %s

typedef unsigned int NSUInteger;
typedef __typeof__(sizeof(int)) size_t;

void *malloc(size_t);
void *calloc(size_t nmemb, size_t size);
void free(void *);

void clang_analyzer_eval(int);

@interface A
- (NSUInteger)foo;
@end

NSUInteger f8(A* x){
  const NSUInteger n = [x foo];
  int* bogus;  

  if (n > 0) {    // tests const cast transfer function logic
    NSUInteger i;
    
    for (i = 0; i < n; ++i)
      bogus = 0;

    if (bogus)  // no-warning
      return n+1;
  }
  
  return n;
}


// PR10163 -- don't warn for default-initialized float arrays.
// (An additional test is in uninit-vals-ps-region.m)
void test_PR10163(float);
void PR10163 (void) {
  float x[2] = {0};
  test_PR10163(x[1]); // no-warning  
}


typedef struct {
  float x;
  float y;
} Point;
typedef struct {
  Point origin;
  int size;
} Circle;

Point makePoint(float x, float y) {
  Point result;
  result.x = x;
  result.y = y;
  return result;
}

void PR14765_test() {
  Circle *testObj = calloc(sizeof(Circle), 1);

  clang_analyzer_eval(testObj->size == 0); // expected-warning{{TRUE}}

  testObj->origin = makePoint(0.0, 0.0);
  if (testObj->size > 0) { ; } // warning occurs here

  // FIXME: Assigning to 'testObj->origin' kills the default binding for the
  // whole region, meaning that we've forgotten that testObj->size should also
  // default to 0. Tracked by <rdar://problem/12701038>.
  // This should be TRUE.
  clang_analyzer_eval(testObj->size == 0); // expected-warning{{UNKNOWN}}

  free(testObj);
}

void PR14765_incorrectBehavior(Circle *testObj) {
  int oldSize = testObj->size;

  clang_analyzer_eval(testObj->size == oldSize); // expected-warning{{TRUE}}

  testObj->origin = makePoint(0.0, 0.0);

  // FIXME: Assigning to 'testObj->origin' kills the default binding for the
  // whole region, meaning that we've forgotten that testObj->size should also
  // default to 0. Tracked by <rdar://problem/12701038>.
  // This should be TRUE.
  clang_analyzer_eval(testObj->size == oldSize); // expected-warning{{UNKNOWN}}

  free(testObj);
}

