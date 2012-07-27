// RUN: %clang_cc1 -analyze -analyzer-checker=core -verify %s

// Test inlining of ObjC class methods.

typedef signed char BOOL;
typedef struct objc_class *Class;
typedef struct objc_object {
    Class isa;
} *id;
@protocol NSObject  - (BOOL)isEqual:(id)object; @end
@interface NSObject <NSObject> {}
+(id)alloc;
-(id)init;
-(id)autorelease;
-(id)copy;
- (Class)class;
-(id)retain;
@end

// Vanila: ObjC class method is called by name.
@interface MyParent : NSObject
+ (int)getInt;
@end
@interface MyClass : MyParent
+ (int)getInt;
@end
@implementation MyClass
+ (int)testClassMethodByName {
    int y = [MyClass getInt];
    return 5/y; // expected-warning {{Division by zero}}
}
+ (int)getInt {
  return 0;
}
@end

// The definition is defined by the parent. Make sure we find it and inline.
@interface MyParentDIP : NSObject
+ (int)getInt;
@end
@interface MyClassDIP : MyParentDIP
@end
@implementation MyClassDIP
+ (int)testClassMethodByName {
    int y = [MyClassDIP getInt];
    return 5/y; // expected-warning {{Division by zero}}
}
@end
@implementation MyParentDIP
+ (int)getInt {
    return 0;
}
@end

// ObjC class method is called by name. Definition is in the category.
@interface AAA : NSObject
@end
@interface AAA (MyCat)
+ (int)getInt;
@end
int foo() {
    int y = [AAA getInt];
    return 5/y; // expected-warning {{Division by zero}}
}
@implementation AAA
@end
@implementation AAA (MyCat)
+ (int)getInt {
    return 0;
}
@end

// There is no declaration in the class but there is one in the parent. Make 
// sure we pick the definition from the class and not the parent.
@interface MyParentTricky : NSObject
+ (int)getInt;
@end
@interface MyClassTricky : MyParentTricky
@end
@implementation MyParentTricky
+ (int)getInt {
    return 0;
}
@end
@implementation MyClassTricky
+ (int)getInt {
  return 1;
}
+ (int)testClassMethodByName {
    int y = [MyClassTricky getInt];
    return 5/y; // no-warning
}
@end

// ObjC class method is called by unknown class declaration (passed in as a 
// parameter). We should not inline in such case.
@interface MyParentUnknown : NSObject
+ (int)getInt;
@end
@interface MyClassUnknown : MyParentUnknown
+ (int)getInt;
@end
@implementation MyClassUnknown
+ (int)testClassVariableByUnknownVarDecl: (Class)cl  {
  int y = [cl getInt];
  return 3/y; // no-warning
}
+ (int)getInt {
  return 0;
}
@end


// False negative.
// ObjC class method call through a decl with a known type.
// We should be able to track the type of currentClass and inline this call.
@interface MyClassKT : NSObject
@end
@interface MyClassKT (MyCatKT)
+ (int)getInt;
@end
@implementation MyClassKT (MyCatKT)
+ (int)getInt {
    return 0;
}
@end
@implementation MyClassKT
- (int)testClassMethodByKnownVarDecl {
  Class currentClass = [self class];
  int y = [currentClass getInt];
  return 5/y; // Would be great to get a warning here.
}
@end
