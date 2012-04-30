#ifndef LLVM_CLANG_SEMA_SIERRA_SOURCE_H
#define LLVM_CLANG_SEMA_SIERRA_SOURCE_H

#include <clang/AST/Type.h>
#include <clang/Basic/SourceLocation.h>
#include <clang/Sema/Ownership.h>

namespace clang {

class Sema;
class AttributeList;

bool CheckSierraSPMDAttr(Sema &S, QualType &type, const AttributeList &attr);

QualType CheckSierraVectorOperands(Sema &S, ExprResult &LHS, ExprResult &RHS, 
                                   SourceLocation Loc, bool IsCompAssign);

QualType BuildSierraVectorType(Sema &S, QualType T, Expr *ArraySize, 
                               SourceLocation AttrLoc);

void HandleSierraVectorAttr(QualType& CurType, 
                            const AttributeList &Attr, Sema &S);

} // end namespace clang

#endif
