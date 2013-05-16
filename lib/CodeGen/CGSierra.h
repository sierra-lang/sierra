//===--- CGSierra.h - Code Generation for Sierra ---------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides the interface for Sierra-related code generation.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_CODEGEN_CGSIERRA_H
#define LLVM_CLANG_CODEGEN_CGSIERRA_H

#include "CGBuilder.h"

namespace llvm {
  class Type;
  class Value;
  class StoreInst;
}

namespace clang {

class IfStmt;
class WhileStmt;
class DoStmt;
class ForStmt;

namespace CodeGen {

class CodeGenFunction;

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

llvm::StoreInst *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask, 
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile);


llvm::Value *EmitMask1ToMask8(CGBuilderTy &Builder, llvm::Value *Mask1);
llvm::Value *EmitMask8ToMask1(CGBuilderTy &Builder, llvm::Value *Mask8);

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S);
void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S);
void EmitSierraDoStmt(CodeGenFunction &CGF, const DoStmt &S);
void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S);

void EmitBranchOnSierraExpr( CodeGenFunction &CGF,
                             const Expr *Cond,
                             bool prefer_true_block,
                             llvm::BasicBlock *TrueBlock,
                             llvm::BasicBlock *FalseBlock );

}  // end namespace CodeGen
}  // end namespace clang

#endif
