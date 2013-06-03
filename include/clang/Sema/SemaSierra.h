//===--- SemaSierra.h - Semantic Analysis Helpers for Sierra ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides the interface for Sierra-related semantic analysis.
//
//===----------------------------------------------------------------------===//

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

QualType CheckSierraVectorLogicalOperands(Sema &S, ExprResult &LHS, ExprResult &RHS,
                                          SourceLocation Loc);

void HandleSierraVectorAttr(Sema &S, QualType& CurType, const AttributeList &Attr);

bool HandleSierraSpmdAttr(Sema &S, const FunctionType *FunT, 
                          const AttributeList &Attr, unsigned &SpmdSize);

} // end namespace clang

#endif
