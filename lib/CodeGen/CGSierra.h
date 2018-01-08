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
class BreakStmt;
class ContinueStmt;
class ReturnStmt;

namespace CodeGen {

class CodeGenFunction;

void createSIMDFunction(CodeGenFunction &);

llvm::Constant *CreateAllOnesVector(llvm::LLVMContext &Context, unsigned NumElems);
llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context, unsigned NumElems);

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

llvm::Value *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask,
                                 llvm::Value *Val, Address Ptr, bool Volatile);

llvm::Value *EmitVecToInt(CGBuilderTy &Builder, llvm::Value *Vec);

llvm::Value *EmitAllTrue (CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAnyTrue (CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value* V);

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S);
void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S);
void EmitSierraDoStmt(CodeGenFunction &CGF, const DoStmt &S);
void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S);
void EmitSierraBreakStmt(CodeGenFunction &CGF, const BreakStmt &S);
void EmitSierraContinueStmt(CodeGenFunction &CGF, const ContinueStmt &S);
void EmitSierraReturnStmt(CodeGenFunction &CGF, const ReturnStmt &S);

/// \brief Holds masks used by the Sierra code generation.
struct SierraMask {
  SierraMask()
    : CurrentMask(nullptr)
    , ContinueMask(nullptr)
  { }

  SierraMask(llvm::Value *CurrentMask, llvm::Value *ContinueMask)
    : CurrentMask(CurrentMask)
    , ContinueMask(ContinueMask)
  {
    assert(CurrentMask  && "must not be null");
    assert(ContinueMask && "must not be null");
  }

  SierraMask(llvm::LLVMContext &Context, unsigned NumElems) {
    assert(0 != NumElems);
    CurrentMask  = CreateAllOnesVector (Context, NumElems);
    ContinueMask = CreateAllZerosVector(Context, NumElems);
  }

  operator bool() { return nullptr != CurrentMask; }

  llvm::Value *CurrentMask;
  llvm::Value *ContinueMask;
};

}
}

#endif
