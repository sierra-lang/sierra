#ifndef LLVM_CLANG_SEMA_SIERRA_SOURCE_H
#define LLVM_CLANG_SEMA_SIERRA_SOURCE_H

#include <clang/AST/Type.h>
#include <clang/Basic/SourceLocation.h>
#include <clang/Sema/Ownership.h>

namespace clang {

class Sema;
class AttributeList;

QualType BuildSierraVectorType(Sema &S, QualType T, Expr *ArraySize, 
                               SourceLocation AttrLoc);

QualType CheckSierraVectorOperands(Sema &S, ExprResult &LHS, ExprResult &RHS, 
                                   SourceLocation Loc, bool IsCompAssign);

void HandleSierraVectorAttr(Sema &S, QualType& CurType, const AttributeList &Attr);

bool HandleSierraSpmdAttr(Sema &S, const FunctionType *FunT, 
                          const AttributeList &Attr, unsigned &SpmdSize);

} // end namespace clang

#endif
