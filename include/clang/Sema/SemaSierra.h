#ifndef LLVM_CLANG_SEMA_SIERRA_SOURCE_H
#define LLVM_CLANG_SEMA_SIERRA_SOURCE_H

#include <clang/AST/Type.h>
#include <clang/Basic/SourceLocation.h>
#include <clang/Sema/Ownership.h>

namespace clang {

class Sema;

QualType CheckSierraVectorOperands(Sema &S, ExprResult &LHS, ExprResult &RHS, 
                                   SourceLocation Loc, bool IsCompAssign);

}

#endif
