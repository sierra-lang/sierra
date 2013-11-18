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

llvm::Constant *CreateAllOnesVector(llvm::LLVMContext &Context, unsigned NumElems);
llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context, unsigned NumElems);

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

llvm::StoreInst *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask, 
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile);

llvm::Value *EmitAllTrue(CGBuilderTy &Builder, llvm::Value *Mask);
llvm::Value *EmitAllFalse(CGBuilderTy &Builder, llvm::Value *Mask);
llvm::Value *EmitAnyTrue(CGBuilderTy &Builder, llvm::Value *Mask);
llvm::Value *EmitAnyFalse(CGBuilderTy &Builder, llvm::Value *Mask);

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S);
void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S);
void EmitSierraDoStmt(CodeGenFunction &CGF, const DoStmt &S);
void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S);

}  // end namespace CodeGen
}  // end namespace clang

#endif
