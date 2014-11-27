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

llvm::Constant *CreateAllOnesVector(llvm::LLVMContext &Context, unsigned NumElems);
llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context, unsigned NumElems);

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

llvm::Value *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask,
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile);

llvm::Value *EmitToInt(CGBuilderTy &Builder, llvm::Value *Vec);

llvm::Value *EmitAllTrue(CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAnyTrue(CodeGenFunction &CGF, llvm::Value* V);
llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value* V);

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S);
void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S);
void EmitSierraDoStmt(CodeGenFunction &CGF, const DoStmt &S);
void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S);
void EmitSierraBreakStmt(CodeGenFunction &CGF, const BreakStmt &S);
void EmitSierraContinueStmt(CodeGenFunction &CGF, const ContinueStmt &S);
void EmitSierraReturnStmt(CodeGenFunction &CGF, const ReturnStmt &S);

// FIXME don't create via new -> it's a memory leak
/// \brief Holds masks used by the Sierra code generation.
class SierraMask {
  // TODO fix private/public
public:
  llvm::Value * const CurrentMask;
  llvm::Value * const ContinueMask;

  static SierraMask * Create( llvm::Value *CurrentMask,
                              llvm::Value *ContinueMask ) {
    assert( CurrentMask );
    assert( ContinueMask );
    return new SierraMask( CurrentMask, ContinueMask );
  }

  static SierraMask * Create( llvm::LLVMContext &Context,
                              unsigned NumElems ) {
    assert( 0 != NumElems );
    return new SierraMask( CreateAllOnesVector( Context, NumElems ),
                           CreateAllZerosVector( Context, NumElems ) );
  }

//private:
  SierraMask( llvm::Value * const CurrentMask,
                     llvm::Value * const ContinueMask )
    : CurrentMask(CurrentMask), ContinueMask(ContinueMask)
  {}
};

}  // end namespace CodeGen
}  // end namespace clang

#endif
